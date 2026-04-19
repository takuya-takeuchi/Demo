# Get started

## Abstracts

* Input csv/tsv files and answer for query if vector database OpenSearch has usable data
  * Embedding and LLM models are via Ollama

## Requirements

### Common

* Python 3.10 or later
* Prepare OpenSearch and Ollama server

## Dependencies

* [LlamaIndex](https://github.com/run-llama/llama_index)
  * v0.14.20
  * MIT license
* [LlamaIndex Llms Integration: Ollama](https://github.com/run-llama/llama_index)
  * v0.10.1
  * MIT license
* [LlamaIndex Embeddings Integration: Ollama](https://github.com/run-llama/llama_index)
  * v0.9.0
  * MIT license
* [LlamaIndex Vector_Stores Integration: Opensearch](https://github.com/run-llama/llama_index)
  * v1.2.0
  * MIT license

## How to use?

At first, creaate virtual python environmental.
Please referr to [LlamaIndex](..).

### Windows

````shell
$ ..\.venv\Scripts\activate
$ python qa.py --embedding_model kun432/cl-nagoya-ruri-large ^
               --embedding_server http://192.168.11.45:11434 ^
               --llm_model gemma4:26b ^
               --llm_server http://192.168.11.45:11434 ^
               --vector_database_server http://192.168.11.45:9200 ^
               --query "2026年のJRA主催愛知杯の勝利騎手と優勝馬について説明して" ^
               --documents_dir documents
Arguments
        embedding_model: kun432/cl-nagoya-ruri-large
       embedding_server: http://192.168.11.45:11434
              llm_model: gemma4:26b
             llm_server: http://192.168.11.45:11434
 vector_database_server: http://192.168.11.45:9200
                  query: 2026年のJRA主催愛知杯の勝利騎手と優勝馬について説明して
              threshold: 0.6
                   rank: 100
            prompt_file: qa_template.txt
          documents_dir: documents
💡 LLM returns good context!!
👤: 2026年のJRA主催愛知杯の勝利騎手と優勝馬について説明して
🤖: 2026年のJRA主催愛知杯（G3）の優勝馬は**アイサンサン**、騎手は**幸英明**です。

$ python qa.py --embedding_model kun432/cl-nagoya-ruri-large ^
               --embedding_server http://192.168.11.45:11434 ^
               --llm_model gemma4:26b ^
               --llm_server http://192.168.11.45:11434 ^
               --vector_database_server http://192.168.11.45:9200 ^
               --query "2026年のヤクルトスワローズの開幕戦について説明して"
Arguments
        embedding_model: kun432/cl-nagoya-ruri-large
       embedding_server: http://192.168.11.45:11434
              llm_model: gemma4:26b
             llm_server: http://192.168.11.45:11434
 vector_database_server: http://192.168.11.45:9200
                  query: 2026年のヤクルトスワローズの開幕戦について説明して
              threshold: 0.6
                   rank: 100
            prompt_file: qa_template.txt
          documents_dir: None
💡 LLM returns good context!!
👤: 2026年のヤクルトスワローズの開幕戦について説明して
🤖: ご提示いただいた参考情報には、2026年の競馬のレース結果に関する情報は含まれていますが、ヤクルトスワローズ（プロ野球）に関する情報は含まれていません。

また、一般的な知識としても、2026年のプロ野球（NPB）の公式日程は現時点ではまだ発表されていません。そのため、2026年のヤクルトスワローズの開幕戦が「いつ」「どの球場で」「どのチームと対戦するか」といっ
た具体的な詳細については、現時点では回答することができません。
````

### Linux and OSX

````shell
$ source ../venv/bin/activate
$ python qa.py --embedding_model kun432/cl-nagoya-ruri-large \
               --embedding_server http://192.168.11.45:11434 \
               --llm_model gemma4:26b \
               --llm_server http://192.168.11.45:11434 \
               --vector_database_server http://192.168.11.45:9200 \
               --query "2026年のヤクルトスワローズの開幕戦について説明して"
````