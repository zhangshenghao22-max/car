import torch
from transformers import AutoModelForCausalLM, AutoTokenizer


model_name = "/data/sdb1/large_models/Qwen/Qwen2.5-1.5B-Instruct"

model = AutoModelForCausalLM.from_pretrained(
    model_name,
    torch_dtype="auto",
    device_map="cpu"
)
tokenizer = AutoTokenizer.from_pretrained(model_name)

special_tokens_list = ["<rtt_0>", "<rtt_1>", "<rtt_2>", "<rtt_3>", "<rtt_end>"]
special_tokens_dict = {"additional_special_tokens": special_tokens_list}

num_added_toks = tokenizer.add_special_tokens(special_tokens_dict)
print("We have added", num_added_toks, "tokens")
# Notice: resize_token_embeddings expect to receive the full size of the new vocabulary, i.e., the length of the tokenizer.
model.resize_token_embeddings(len(tokenizer))

with torch.no_grad():
    embeddings = model.get_input_embeddings().weight
    # 使用已有token的均值初始化
    new_emb_mean = embeddings[:-len(special_tokens_dict["additional_special_tokens"])].mean(dim=0)
    # 添加微小随机扰动
    new_embeddings = new_emb_mean + torch.randn(len(special_tokens_dict["additional_special_tokens"]), 
                                               embeddings.size(1)) * 0.001
    embeddings[-len(special_tokens_dict["additional_special_tokens"]):] = new_embeddings


save_path = "/data/sdb1/large_models/Qwen/Qwen2.5-1.5B-Instruct_add_token"
# 保存模型
tokenizer.save_pretrained(save_path)
model.save_pretrained(save_path)


# 测试
for i in special_tokens_list:
    print(f"{i} id: {tokenizer.convert_tokens_to_ids(i)}")










