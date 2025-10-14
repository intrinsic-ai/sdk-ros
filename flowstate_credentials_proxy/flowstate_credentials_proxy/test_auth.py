import json
import unittest
from unittest.mock import MagicMock, mock_open, patch

import requests

from flowstate_credentials_proxy.auth import InvalidOrganizationError, TokenSource


class TestTokenSource(unittest.TestCase):
    mock_user_token = {
        "name": "test_project",
        "tokens": {"default": {"apiKey": "test-api-key"}},
    }

    @patch("pathlib.Path.open", new_callable=mock_open)
    @patch("flowstate_credentials_proxy.auth.requests.post")
    def test_get_access_token_success(
        self, mock_post: MagicMock, mock_open_file: MagicMock
    ) -> None:
        mock_open_file.return_value.read.return_value = json.dumps(self.mock_user_token)

        # {
        #     "iss": "test",
        #     "sub": "test",
        #     "aud": "test",
        #     "iat": 0,
        #     "exp": 2177596800 # 2039-01-01
        # }
        test_token = "eyJhbGciOiJub25lIiwidHlwIjoiSldUIn0.eyJzdWIiOiIxMjM0NTY3ODkwIiwibmFtZSI6IkpvaG4gRG9lIiwiYWRtaW4iOnRydWUsImlhdCI6MCwiZXhwIjoyMTc3NTk2ODAwfQ."

        mock_response = MagicMock()
        mock_response.json.return_value = {"idToken": test_token}
        mock_response.raise_for_status.return_value = None
        mock_post.return_value = mock_response

        ts = TokenSource("user@project")
        access_token = ts.get_token()

        self.assertEqual(access_token, test_token)
        mock_post.assert_called_once_with(
            "https://flowstate.intrinsic.ai/api/v1/accountstokens:idtoken",
            headers={"content-type": "application/json"},
            json={
                "apiKey": "test-api-key",
                "do_fan_out": False,
                "api_key_project_hint": "project",
            },
        )

    def test_invalid_organization(self) -> None:
        with self.assertRaises(InvalidOrganizationError) as e:
            TokenSource("invalid-organization")
        self.assertEqual(str(e.exception), "invalid organization")

    @patch("pathlib.Path.open", new_callable=mock_open)
    def test_invalid_json(self, mock_open_file: MagicMock) -> None:
        mock_open_file.return_value.read.return_value = "invalid json"
        with self.assertRaises(json.JSONDecodeError):
            TokenSource("user@project")

    @patch("pathlib.Path.open", new_callable=mock_open)
    def test_missing_key(self, mock_open_file: MagicMock) -> None:
        mock_open_file.return_value.read.return_value = json.dumps({})
        with self.assertRaises(KeyError):
            TokenSource("user@project")

    @patch("pathlib.Path.open", new_callable=mock_open)
    @patch("flowstate_credentials_proxy.auth.requests.post")
    def test_get_token_http_error(
        self, mock_post: MagicMock, mock_open_file: MagicMock
    ) -> None:
        mock_open_file.return_value.read.return_value = json.dumps(self.mock_user_token)
        mock_response = MagicMock()
        mock_response.raise_for_status.side_effect = requests.exceptions.HTTPError
        mock_post.return_value = mock_response

        with self.assertRaises(requests.exceptions.HTTPError):
            TokenSource("user@project").get_token()

    @patch("pathlib.Path.open", new_callable=mock_open)
    @patch("flowstate_credentials_proxy.auth.requests.post")
    def test_get_access_token_missing_key(
        self, mock_post: MagicMock, mock_open_file: MagicMock
    ) -> None:
        mock_open_file.return_value.read.return_value = json.dumps(self.mock_user_token)
        mock_response = MagicMock()
        mock_response.json.return_value = {}
        mock_response.raise_for_status.return_value = None
        mock_post.return_value = mock_response

        with self.assertRaises(KeyError):
            TokenSource("user@project").get_token()


if __name__ == "__main__":
    unittest.main()
