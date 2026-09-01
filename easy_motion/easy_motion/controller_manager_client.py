"""ROS 2 client utilities for managing ros2_control controllers."""

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration

from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import (
    ListControllers,
    LoadController,
    ConfigureController,
    SwitchController,
    UnloadController,
)


class ControllerManagerClient(Node):
    """Client for interacting with a ros2_control controller manager.

    Supported operations include:

    - Listing controllers and inspecting their state.
    - Loading, configuring, activating, and deactivating controllers.
    - Switching multiple controllers in a single request.
    - Unloading controllers.
    - Finding controllers that command a given set of joints.
    """

    def __init__(
        self,
        controller_manager_name: str = "/controller_manager",
    ):
        """Initialize the controller manager client.

        Args:
            controller_manager_name: Fully qualified name of the target controller
                manager node.

        Raises:
            RuntimeError: If a required controller manager service is unavailable.
        """
        super().__init__(
            "controller_manager_client_node",
            use_global_arguments=False,
        )

        self.declare_parameter("controller_manager_name", controller_manager_name)

        self.controller_manager_name = (
            self.get_parameter("controller_manager_name")
            .get_parameter_value()
            .string_value
        )


        self.list_controllers_client = self.create_client(
            ListControllers,
            f"{self.controller_manager_name}/list_controllers",
        )

        self.load_controller_client = self.create_client(
            LoadController,
            f"{self.controller_manager_name}/load_controller",
        )

        self.configure_controller_client = self.create_client(
            ConfigureController,
            f"{self.controller_manager_name}/configure_controller",
        )

        self.switch_controller_client = self.create_client(
            SwitchController,
            f"{self.controller_manager_name}/switch_controller",
        )

        self.unload_controller_client = self.create_client(
            UnloadController,
            f"{self.controller_manager_name}/unload_controller",
        )

        self._wait_for_core_services()

    def _wait_for_core_services(self):
        """Wait for the controller manager services required by this client.

        Raises:
            RuntimeError: If a required service is unavailable after five seconds.
        """
        services = [
            (self.list_controllers_client, "list_controllers"),
            (self.load_controller_client, "load_controller"),
            (self.configure_controller_client, "configure_controller"),
            (self.switch_controller_client, "switch_controller"),
            (self.unload_controller_client, "unload_controller"),
        ]

        for client, name in services:
            if not client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(
                    f"Controller manager service not available: "
                    f"{self.controller_manager_name}/{name}"
                )

    def _call(self, client, request, service_name: str):
        """Call a controller manager service synchronously.

        Args:
            client: ROS 2 service client used to send the request.
            request: Request message accepted by the service.
            service_name: Name of the service being called.

        Returns:
            The service response, or ``None`` if the call does not complete within
            five seconds.
        """
        future = client.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=5.0,
        )

        if not future.done():
            return None

        response = future.result()

        return response

    def list_controllers(self) -> list[ControllerState]:
        """List the controllers known to the controller manager.

        Returns:
            Controller states reported by the controller manager, or an empty list
            if the service call times out.
        """
        request = ListControllers.Request()

        response = self._call(
            self.list_controllers_client,
            request,
            "list_controllers",
        )

        if response is None:
            self.get_logger().error(
                f"Timeout while calling list_controllers. "
                f"Make sure the controller manager is running and the service is available."
            )
            return []

        return response.controller

    def get_controller(self, controller_name: str) -> ControllerState | None:
        """Get information about a controller.

        Args:
            controller_name: Name of the controller to find.

        Returns:
            The matching controller state, or ``None`` if the controller is unknown.
        """
        for controller in self.list_controllers():
            if controller.name == controller_name:
                return controller

        return None

    def is_loaded(self, controller_name: str) -> bool:
        """Check whether a controller is loaded.

        Args:
            controller_name: Name of the controller to check.

        Returns:
            ``True`` if the controller is known to the controller manager.
        """
        return self.get_controller(controller_name) is not None

    def is_active(self, controller_name: str) -> bool:
        """Check whether a controller is active.

        Args:
            controller_name: Name of the controller to check.

        Returns:
            ``True`` if the controller exists and its state is ``active``.
        """
        controller = self.get_controller(controller_name)

        if controller is None:
            return False

        return controller.state == "active"

    def load_controller(self, controller_name: str) -> bool:
        """Load a controller if it is not already loaded.

        Args:
            controller_name: Name of the controller to load.

        Returns:
            ``True`` if the controller is already loaded or is loaded successfully;
            ``False`` if the service times out or rejects the request.
        """
        if self.is_loaded(controller_name):
            return True

        request = LoadController.Request()
        request.name = controller_name

        response = self._call(
            self.load_controller_client,
            request,
            "load_controller",
        )

        if response is None:
            self.get_logger().error(
                f"Timeout while calling load_controller for: {controller_name}. "
                f"Make sure the controller manager is running and the service is available."
            )
            return False
        
        if not response.ok:
            self.get_logger().error(
                f"Failed to load controller: {controller_name}. ")
            return False

        return True

    def configure_controller(self, controller_name: str) -> bool:
        """Configure a controller if necessary.

        Args:
            controller_name: Name of the controller to configure.

        Returns:
            ``True`` if the controller is already inactive or active, or if it is
            configured successfully; ``False`` if the service times out or rejects
            the request.
        """
        controller = self.get_controller(controller_name)

        if controller is not None and controller.state in ["inactive", "active"]:
            return True

        request = ConfigureController.Request()
        request.name = controller_name

        response = self._call(
            self.configure_controller_client,
            request,
            "configure_controller",
        )

        if response is None:
            self.get_logger().error(
                f"Timeout while calling configure_controller for: {controller_name}. "
                f"Make sure the controller manager is running and the service is available."
            )
            return False

        if not response.ok:
            self.get_logger().error(
                f"Failed to configure controller: {controller_name}. ")
            return False
            
        return True

    def activate_controller(
        self,
        controller_name: str,
        strict: bool = True,
        timeout_sec: float = 5.0,
    ) -> bool:
        """Activate a single controller.

        Args:
            controller_name: Name of the controller to activate.
            strict: Whether any controller switch failure should fail the request.
            timeout_sec: Maximum time allowed by the controller manager for the
                switch operation.

        Returns:
            Whether the controller switch succeeded.
        """
        return self.switch_controllers(
            activate=[controller_name],
            deactivate=[],
            strict=strict,
            timeout_sec=timeout_sec,
        )

    def deactivate_controller(
        self,
        controller_name: str,
        strict: bool = True,
        timeout_sec: float = 5.0,
    ) -> bool:
        """Deactivate a single controller.

        Args:
            controller_name: Name of the controller to deactivate.
            strict: Whether any controller switch failure should fail the request.
            timeout_sec: Maximum time allowed by the controller manager for the
                switch operation.

        Returns:
            Whether the controller switch succeeded.
        """
        return self.switch_controllers(
            activate=[],
            deactivate=[controller_name],
            strict=strict,
            timeout_sec=timeout_sec,
        )

    def load_configure_switch_controllers(
        self,
        activate: str | list[str],
        deactivate: list[str] | None = None,
        load: bool = True,
        configure: bool = True,
        strict: bool = True,
        timeout_sec: float = 5.0,
    ) -> bool:
        """Prepare controllers and switch them in a single workflow.

        Controllers selected for activation are optionally loaded and configured
        before the switch request is sent.

        Args:
            activate: Controller name or names to activate.
            deactivate: Controller names to deactivate during the switch.
            load: Whether to load the controllers selected for activation.
            configure: Whether to configure the controllers selected for activation.
            strict: Whether any controller switch failure should fail the request.
            timeout_sec: Maximum time allowed by the controller manager for the
                switch operation.

        Returns:
            ``True`` if every requested step succeeds; otherwise ``False``.
        """
        if isinstance(activate, str):
            activate = [activate]

        deactivate = deactivate or []
        # The pipeline should be: load -> configure -> switch
        if load:
            for controller_name in activate:
                if not self.load_controller(controller_name):
                    return False

        if configure:
            for controller_name in activate:
                if not self.configure_controller(controller_name):
                    return False

        return self.switch_controllers(
            activate=activate,
            deactivate=deactivate,
            strict=strict,
            timeout_sec=timeout_sec,
        )

    def switch_controllers(
        self,
        activate: list[str],
        deactivate: list[str],
        strict: bool = True,
        activate_asap: bool = True,
        timeout_sec: float = 5.0,
    ) -> bool:
        """Activate and deactivate controllers in a single switch request.

        Args:
            activate: Controller names to activate.
            deactivate: Controller names to deactivate.
            strict: Use strict switching when ``True``; otherwise use best-effort
                switching.
            activate_asap: Activate controllers as soon as their hardware interfaces
                become available, without waiting for all requested controllers.
            timeout_sec: Maximum time allowed by the controller manager for the
                switch operation.

        Returns:
            ``True`` if the switch succeeds; ``False`` if the service times out or
            rejects the request.
        """
        request = SwitchController.Request()

        request.activate_controllers = activate
        request.deactivate_controllers = deactivate
        request.strictness = (
            SwitchController.Request.STRICT
            if strict
            else SwitchController.Request.BEST_EFFORT
        )
        request.activate_asap = activate_asap
        request.timeout = self._duration_from_seconds(timeout_sec)

        response = self._call(
            self.switch_controller_client,
            request,
            "switch_controller",
        )

        if response is None:
            self.get_logger().error(
                f"Timeout while calling switch_controller. "
                f"Make sure the controller manager is running and the service is available."
            )
            return False

        if not response.ok:
            self.get_logger().error(
                "Failed to switch controllers."
            )
            return False
        
        return True

    def unload_controller(
        self,
        controller_name: str,
        deactivate_if_active: bool = True,
    ) -> bool:
        """Unload a controller.

        Args:
            controller_name: Name of the controller to unload.
            deactivate_if_active: Whether to request deactivation before unloading an
                active controller.

        Returns:
            ``True`` if the controller is unloaded successfully; ``False`` if the
            service times out or rejects the request.
        """
        if deactivate_if_active and self.is_active(controller_name):
            self.deactivate_controller(controller_name)

        request = UnloadController.Request()
        request.name = controller_name

        response = self._call(
            self.unload_controller_client,
            request,
            "unload_controller",
        )

        if response is None:
            self.get_logger().error(
                f"Timeout while calling unload_controller for: {controller_name}. "
                f"Make sure the controller manager is running and the service is available."
            )
            return False

        if not response.ok:
            self.get_logger().error(
                f"Failed to unload controller: {controller_name}"
            )
            return False
        
        return True
    
    def _interface_to_joint_name(self, interface_name: str) -> str:
        """Extract a joint name from a ``joint/interface`` resource name.

        Args:
            interface_name: Fully qualified hardware interface name.

        Returns:
            The portion preceding the first ``/`` separator.
        """
        return interface_name.split("/")[0]

    def get_controller_command_joints(self, controller_name: str) -> list[str]:
        """Get the joints commanded by a controller.

        Required command interfaces are preferred when available; claimed interfaces
        are used as a fallback.

        Args:
            controller_name: Name of the controller to inspect.

        Returns:
            Sorted unique joint names, or an empty list if the controller is unknown
            or exposes no command interfaces.
        """
        controller = self.get_controller(controller_name)

        if controller is None:
            return []

        interfaces = list(controller.required_command_interfaces)

        if not interfaces:
            interfaces = list(controller.claimed_interfaces)

        joints = sorted({
            self._interface_to_joint_name(interface)
            for interface in interfaces
            if "/" in interface
        })

        return joints

    def get_controller_infos_for_joints(
        self,
        joints: list[str],
        active_only: bool = False,
        exact_match: bool = False,
    ) -> list[ControllerState]:
        """Find controllers that command a requested set of joints.

        Args:
            joints: Joint names that a matching controller must command.
            active_only: Whether to consider only active controllers.
            exact_match: Require the controller joint set to equal ``joints`` when
                ``True``. Otherwise, accept controllers whose joint set contains all
                requested joints.

        Returns:
            Controller states satisfying the requested filters.
        """
        requested_joints = set(joints)
        matching_controllers = []

        for controller in self.list_controllers():
            if active_only and controller.state != "active":
                continue

            controller_joints = set(
                self.get_controller_command_joints(controller.name)
            )

            if not controller_joints:
                continue

            if exact_match:
                match = controller_joints == requested_joints
            else:
                match = requested_joints.issubset(controller_joints)

            if match:
                matching_controllers.append(controller)

        return matching_controllers

    @staticmethod
    def _duration_from_seconds(seconds: float) -> Duration:
        """Convert floating-point seconds to a ROS 2 duration message.

        Args:
            seconds: Duration expressed in seconds.

        Returns:
            The corresponding ROS 2 duration message.
        """
        duration = Duration()
        duration.sec = int(seconds)
        duration.nanosec = int((seconds - int(seconds)) * 1e9)
        return duration
