import google.auth.transport.requests
from google.oauth2 import service_account

def _get_access_token(json):
  SCOPES = ['https://www.googleapis.com/auth/firebase.messaging']
  credentials = service_account.Credentials.from_service_account_file(json, scopes=SCOPES)
  request = google.auth.transport.requests.Request()
  credentials.refresh(request)
  return credentials.token

access_token = _get_access_token('service-account.json')
print(f"access token: {access_token}")