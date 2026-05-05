import os
from dotenv import load_dotenv
from google import genai
load_dotenv('src/image_to_llm/llm_config.env')
print("HTTP_PROXY:", os.environ.get("HTTP_PROXY"))
print("HTTPS_PROXY:", os.environ.get("HTTPS_PROXY"))
client = genai.Client(api_key=os.environ.get("GEMINI_API_KEY"))
print("Ready")
try:
    response = client.models.generate_content(model="gemini-2.5-flash", contents="hello")
    print(response.text)
except Exception as e:
    print("Error:", e)
