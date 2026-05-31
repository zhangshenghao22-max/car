import openai
 
client = openai.OpenAI(api_key="ytl2gA0DpJgveobtA9Ca4aF2-3CeD-4844-8E17-123b80B6", base_url="https://api.modelverse.cn/v1/")
stream = client.chat.completions.create(
    model="deepseek-ai/DeepSeek-V3.2",
    messages=[{"role": "user", "content": "Hello!"}],
    stream=True
)
for chunk in stream:
    if chunk.choices[0].delta.content:
        print(chunk.choices[0].delta.content, end="")