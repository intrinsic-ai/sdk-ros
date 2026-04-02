import unittest
from concurrent import futures

import grpc
from intrinsic.frontend.solution_service.proto.solution_service_pb2 import \
    GetStatusRequest
from intrinsic.frontend.solution_service.proto.solution_service_pb2_grpc import (
    SolutionServiceServicer, SolutionServiceStub,
    add_SolutionServiceServicer_to_server)
from intrinsic.frontend.solution_service.proto.status_pb2 import Status


class TestSolutionService(SolutionServiceServicer):
    def GetStatus(self, request, context):
        return Status()


class TestGrpc(unittest.TestCase):
    def test_grpc(self):
        server = grpc.server(futures.ThreadPoolExecutor(max_workers=1))
        add_SolutionServiceServicer_to_server(TestSolutionService(), server)
        port = server.add_insecure_port("[::]:0")
        server.start()

        try:
            with grpc.insecure_channel(f"localhost:{port}") as channel:
                stub = SolutionServiceStub(channel)
                response = stub.GetStatus(GetStatusRequest())
                self.assertIsInstance(response, Status)
        finally:
            server.stop(None)
