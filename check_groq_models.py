import os
import json
import urllib.request

api_key = os.environ.get("GROQ_API_KEY")
if not api_key:
    print("ERROR: GROQ_API_KEY not set.")
    exit(1)

req = urllib.request.Request("https://api.groq.com/openai/v1/models")
req.add_header("Authorization", f"Bearer {api_key}")
try:
    with urllib.request.urlopen(req) as response:
        data = json.loads(response.read().decode())
        print("Available Groq Models:")
        for model in data.get("data", []):
            print(f" - {model['id']}")
except Exception as e:
    print(f"Failed to fetch models: {e}")
