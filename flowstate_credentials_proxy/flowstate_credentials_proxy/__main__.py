import argparse
import sys

from flowstate_credentials_proxy import auth
from flowstate_credentials_proxy.auth import InvalidOrganizationError


def main():
    parser = argparse.ArgumentParser(
        prog="flowstate_credentials_proxy",
        description="Start a websocket proxy to allow a local zenoh router to connect to flowstate.",
    )
    parser.add_argument(
        "--org",
        required=True,
        help="The organization name in the format organization@project.",
    )
    args = parser.parse_args()

    try:
        api_key = auth.get_api_key(args.org)
        project = auth.get_project_from_organization(args.org)
        token = auth.get_access_token(project, api_key)
    except FileNotFoundError:
        print(
            f"""Error: Credentials for given organization "{args.org}" not found.
Run 'inctl auth login --org {args.org}' to add it.

Download inctl here https://github.com/intrinsic-ai/sdk/releases""",
            file=sys.stderr,
        )
    except InvalidOrganizationError:
        print("Organization is invalid", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
