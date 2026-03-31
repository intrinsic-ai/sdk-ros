import unittest
import grpc
import grpc_testing

import test_pb2
import test_pb2_grpc

class TestServiceServicer(test_pb2_grpc.TestServiceServicer):
    def Ping(self, request, context):
        return test_pb2.PingResponse(message=f"Pong: {request.message}")

class TestGrpc(unittest.TestCase):
    def setUp(self):
        self._servicer = TestServiceServicer()
        self._test_server = grpc_testing.server_from_dictionary(
            {
                test_pb2.DESCRIPTOR.services_by_name['TestService']: self._servicer
            },
            grpc_testing.strict_real_time()
        )

    def test_ping(self):
        request = test_pb2.PingRequest(message='world')
        method = test_pb2.DESCRIPTOR.services_by_name['TestService'].methods_by_name['Ping']

        rpc = self._test_server.invoke_unary_unary(method, (), request, None)

        response, trailing_metadata, code, details = rpc.termination()

        self.assertEqual(code, grpc.StatusCode.OK)
        self.assertEqual(response.message, 'Pong: world')

if __name__ == '__main__':
    unittest.main()
