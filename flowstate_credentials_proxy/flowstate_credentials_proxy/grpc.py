import grpc


def create_channel(project: str) -> grpc.Channel:
    channel = grpc.secure_channel(
        f"www.endpoints.{project}.cloud.goog:443",
        grpc.ssl_channel_credentials(),
        options=[("grpc.max_receive_message_length", -1)],
    )
    return channel
