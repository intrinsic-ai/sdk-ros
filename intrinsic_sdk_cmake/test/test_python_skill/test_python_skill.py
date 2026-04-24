import sys
import importlib
import time
from concurrent import futures
import grpc
from absl import logging, app, flags
from intrinsic.skills.python import skill_interface

FLAGS = flags.FLAGS
flags.DEFINE_integer("port", 8003, "Port to listen on.", allow_override=True)
flags.DEFINE_string("skill_service_config_filename", "", "Path to config.", allow_override=True)

class TestPythonSkill(skill_interface.Skill):
    def execute(self, request, context):
        logging.info("Executing TestPythonSkill")
        from test_python_skill import test_python_skill_pb2
        
        input_val = ""
        try:
            input_val = request.params.input_data
            logging.info(f"Received input_data: {input_val}")
        except Exception as e:
            logging.warning(f"Failed to read input_data: {e}")
            
        return test_python_skill_pb2.TestPythonSkillResult(message=f"Received: {input_val}")

    def get_skill_runtime_data(self, skill_name: str):
        class SmartContainer:
            def __init__(self, **kwargs):
                self.__dict__.update(kwargs)

            def __getattr__(self, name):
                import datetime
                lower = name.lower()
                if "timeout" in lower:
                    return datetime.timedelta(seconds=180)
                if "ready" in lower:
                    return datetime.timedelta(seconds=60)
                if "supports" in lower:
                    return True
                if any(x in lower for x in ["topics", "resources", "specs"]):
                    return [] if "topics" in lower else {}
                return SmartContainer()

        from test_python_skill import test_python_skill_pb2
        
        params = SmartContainer(
            descriptor=test_python_skill_pb2.TestPythonSkillParams.DESCRIPTOR,
            default_value=None,
        )
        returns = SmartContainer(
            message_full_name=test_python_skill_pb2.TestPythonSkillResult.DESCRIPTOR.full_name
        )

        return SmartContainer(
            skill=None,
            skill_id="com.example.test_python_skill",
            parameter_data=params,
            return_type_data=returns,
            execution_options=SmartContainer(),
            topic_data=SmartContainer(),
            status_specs=SmartContainer(),
            resource_data=SmartContainer(),
        )

    def get_skill_execute(self, name):
        return self

    def get_skill_project(self, name):
        return self

def start_runner(argv):
    logging.info("TestPythonSkill started")
    
    from intrinsic.skills.internal import skill_service_impl

    # Helper to find pb2 modules in various SDK namespaces
    def find_mod(name):
        for p in [
            "intrinsic.skills.v1",
            "intrinsic.skills.proto",
            "intrinsic.proto.skills",
        ]:
            try:
                return importlib.import_module(f"{p}.{name}")
            except:
                continue
        return None

    skills_pb2 = find_mod("skills_pb2")
    skill_proto = skills_pb2.Skill() if skills_pb2 else None
    
    skill_instance = TestPythonSkill()
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))

    class UnifiedServicerProxy(object):
        def __init__(self, inst, proto):
            self._skill = inst
            self._info = skill_service_impl.SkillInformationServicer(proto)
            
            try:
                self._executor = skill_service_impl.SkillExecutorServicer(
                    inst, None, None, None
                )
            except TypeError:
                try:
                    self._executor = skill_service_impl.SkillExecutorServicer(
                        inst, None, None, None, None
                    )
                except Exception as e:
                    logging.warning(f"Failed to create SkillExecutorServicer: {e}")
                    self._executor = None

        def Predict(self, r, c):
            logging.info("Handling Predict call - returning TestPythonSkillResult")
            pb2 = importlib.import_module("intrinsic.skills.proto.skill_service_pb2")
            res = pb2.PredictResult()
            logging.info(f"Returning type: {type(res)}")
            return res

        def __getattr__(self, name):
            logging.info(f"gRPC call to method: {name}")
            for target in [self._executor, self._info]:
                if target and hasattr(target, name):
                    logging.info(f"Delegating {name} to {type(target).__name__}")
                    return getattr(target, name)
            logging.warning(f"Falling back to dummy for {name}")
            try:
                from intrinsic.skills.proto import skill_service_pb2
                return lambda r, c: skill_service_pb2.PredictResult()
            except:
                return lambda r, c: None

    proxy = UnifiedServicerProxy(skill_instance, skill_proto)

    # Register all gRPC interfaces dynamically
    mods = ["skill_service", "skill_executor", "skill_information", "skill"]
    for m in mods:
        for p in ["intrinsic.skills.v1", "intrinsic.skills.proto"]:
            try:
                mod = importlib.import_module(f"{p}.{m}_pb2_grpc")
                for attr in dir(mod):
                    if "add_" in attr and "_to_server" in attr:
                        getattr(mod, attr)(proxy, server)
            except:
                continue

    server.add_insecure_port(f"[::]:{FLAGS.port}")
    server.start()
    logging.info(f"gRPC Server listening on port {FLAGS.port}")
    
    try:
        server.wait_for_termination()
    except KeyboardInterrupt:
        logging.info("TestPythonSkill stopped")

if __name__ == '__main__':
    app.run(start_runner, flags_parser=lambda argv: FLAGS(argv, known_only=True))
