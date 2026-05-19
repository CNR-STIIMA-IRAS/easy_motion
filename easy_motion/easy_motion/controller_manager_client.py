import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration

from controller_manager_msgs.srv import (
    ListControllers,
    LoadController,
    ConfigureController,
    SwitchController,
    UnloadController,
)


class ControllerManagerClient(Node):
    def __init__(
        self,
        controller_manager_name: str = "/controller_manager",
    ):
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

    def _call(self, client, request, service_name: str) -> rclpy.client.ClientResponse | None:
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

    def list_controllers(self) -> list[ListControllers.Response.ControllerState]:
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

    def get_controller(self, controller_name: str) -> ListControllers.Response.ControllerState | None:
        for controller in self.list_controllers():
            if controller.name == controller_name:
                return controller

        return None

    def is_loaded(self, controller_name: str) -> bool:
        return self.get_controller(controller_name) is not None

    def is_active(self, controller_name: str) -> bool:
        controller = self.get_controller(controller_name)

        if controller is None:
            return False

        return controller.state == "active"

    def load_controller(self, controller_name: str) -> bool:
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
    
    def get_controller_infos_for_joints(
        self,
        joints: list[str],
        active_only: bool = False,
        exact_match: bool = False,
    ) -> list[ListControllers.Response.ControllerState]:
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
        duration = Duration()
        duration.sec = int(seconds)
        duration.nanosec = int((seconds - int(seconds)) * 1e9)
        return duration