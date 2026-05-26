# flowstate_credentials_proxy

`flowstate_credentials_proxy` is a WebSocket proxy designed to facilitate the connection between a local Zenoh router and Flowstate. It handles the necessary authentication by leveraging `inctl` credentials to obtain access tokens and then proxies the traffic to the appropriate Flowstate endpoint.

## Prerequisites

- **inctl:** The Intrinsic command-line tool must be installed.
  - Download `inctl` from [GitHub Releases](https://github.com/intrinsic-ai/sdk/releases).
- **Authentication:** You must be logged into your organization using `inctl`.
  - Run: `inctl auth login --org <organization>@<project>`

## Setup

### Adding the Flowstate ROS Bridge Service

Before using the proxy, ensure the `flowstate_ros_bridge` service is added to your Flowstate solution:

```bash
inctl service add ai.intrinsic.flowstate_ros_bridge --cluster <cluster> --name <service_name> --org <flowstate_org>
```

The `<cluster>` name can be found in the Flowstate UI or by examining the solution editor URL:
`https://flowstate.intrinsic.ai/solution-editor/:organization/:cluster/`

## Usage

You can start the proxy using either the `credentials_proxy` script (if installed as a ROS package) or by running the module directly.

### Running via ROS 2

```bash
ros2 run flowstate_credentials_proxy credentials_proxy --org <org>@<project> --cluster <cluster> --service <service>
```

### Arguments

- `--org`: (Required) The organization name in the format `organization@project`.
- `--cluster`: (Required) The name of the Flowstate cluster to use.
- `--service`: (Required) The name of the `flowstate_ros_bridge` instance.
- `--port`: (Optional) The port to listen on. Defaults to `7448`.

## Integration with Zenoh

Once the proxy is running, you can connect a local `rmw_zenohd` router to it by overriding its configuration.

```bash
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["ws/localhost:7448"];routing/router/peers_failover_brokering=true'
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Replace `7448` with the custom port if you specified one when starting the proxy.
