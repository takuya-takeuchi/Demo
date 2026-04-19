# -*- coding: utf-8 -*-
#!/usr/bin/python
import argparse
import asyncio
import csv
import os
from pathlib import Path

from llama_index.core import Document, SimpleDirectoryReader, StorageContext, VectorStoreIndex
from llama_index.core.prompts import PromptTemplate
from llama_index.embeddings.ollama import OllamaEmbedding
from llama_index.llms.ollama import Ollama
from llama_index.vector_stores.opensearch import (
    OpensearchVectorClient,
    OpensearchVectorStore,
)

def detect_csv_dialect(sample_text: str) -> csv.Dialect:
    try:
        return csv.Sniffer().sniff(sample_text, delimiters=[",", "\t", ";"])
    except csv.Error:
        class DefaultDialect(csv.excel):
            delimiter = ","
        return DefaultDialect()

def row_to_text(row: dict[str, str]) -> str:
    parts = []
    for key, value in row.items():
        key = "" if key is None else str(key).strip()
        value = "" if value is None else str(value).strip()
        if key:
            parts.append(f"{key}: {value}")
    return "、".join(parts)

def clean_row(row: dict) -> dict[str, str]:
    cleaned = {}
    for key, value in row.items():
        if key is None:
            continue

        k = str(key).strip()
        if not k:
            continue

        v = "" if value is None else str(value).strip()
        cleaned[k] = v

    return cleaned

def load_tabular_file_as_documents(path: str) -> list[Document]:
    file_path = Path(path)

    with open(file_path, "r", encoding="utf-8-sig", newline="") as f:
        sample = f.read(4096)
        f.seek(0)

        dialect = detect_csv_dialect(sample)
        reader = csv.DictReader(f, dialect=dialect)

        documents = []
        for i, row in enumerate(reader, start=1):
            metadata = clean_row(row)

            if not metadata:
                continue

            text = row_to_text(metadata)

            doc = Document(
                text=text,
                metadata=metadata,
            )
            documents.append(doc)

    return documents

def load_documents(documents_dir: str):
    documents = []

    tabular_files = []
    other_files = []

    for path in Path(documents_dir).rglob("*"):
        if not path.is_file():
            continue

        if path.suffix.lower() in [".csv", ".tsv"]:
            tabular_files.append(path)
        else:
            other_files.append(path)

    for path in tabular_files:
        documents.extend(load_tabular_file_as_documents(str(path)))

    if other_files:
        other_docs = SimpleDirectoryReader(
            input_files=[str(p) for p in other_files]
        ).load_data(show_progress=True)
        documents.extend(other_docs)

    return documents

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--embedding_model", type=str, required=True)
    parser.add_argument("-s", "--embedding_server", type=str, required=True)
    parser.add_argument("-l", "--llm_model", type=str, required=True)
    parser.add_argument("-r", "--llm_server", type=str, required=True)
    parser.add_argument("-v", "--vector_database_server", type=str, required=True)
    parser.add_argument("-q", "--query", type=str, required=True)
    parser.add_argument("-t", "--threshold", type=float, default=0.6)
    parser.add_argument("-k", "--rank", type=int, default=100)
    parser.add_argument("-p", "--prompt_file", type=str, default="qa_template.txt")
    parser.add_argument("-d", "--documents_dir", type=str)
    return parser.parse_args()

if __name__ == '__main__':
    args = get_args()
    embedding_model        = args.embedding_model
    embedding_server       = args.embedding_server
    llm_model              = args.llm_model
    llm_server             = args.llm_server
    vector_database_server = args.vector_database_server
    query                  = args.query
    threshold              = args.threshold
    rank                   = args.rank
    prompt_file            = args.prompt_file
    documents_dir          = args.documents_dir

    print("Arguments")
    print("        embedding_model: {}".format(embedding_model))
    print("       embedding_server: {}".format(embedding_server))
    print("              llm_model: {}".format(llm_model))
    print("             llm_server: {}".format(llm_server))
    print(" vector_database_server: {}".format(vector_database_server))
    print("                  query: {}".format(query))
    print("              threshold: {}".format(threshold))
    print("                   rank: {}".format(rank))
    print("            prompt_file: {}".format(prompt_file))
    print("          documents_dir: {}".format(documents_dir))

    embedding_model = OllamaEmbedding(model_name=embedding_model, base_url=embedding_server)
    llm_model = Ollama(model=llm_model, base_url=llm_server, request_timeout=120.0)

    index = "demo"
    dim = 1024
    text_field = "content"
    embedding_field = "embedding"

    client = None
    vector_store = None

    try:
        with open(prompt_file, "r", encoding="utf-8") as f:
            prompt_text = f.read()

        client = OpensearchVectorClient(
            endpoint=vector_database_server,
            index=index,
            dim=dim,
            embedding_field=embedding_field,
            text_field=text_field,
        )

        vector_store = OpensearchVectorStore(client)

        if documents_dir and os.path.exists(documents_dir):
            documents = load_documents(documents_dir)
            storage_context = StorageContext.from_defaults(vector_store=vector_store)
            index = VectorStoreIndex.from_documents(
                documents=documents,
                storage_context=storage_context,
                embed_model=embedding_model,
                show_progress=False,
            )
        else:
            index = VectorStoreIndex.from_vector_store(
                vector_store=vector_store,
                embed_model=embedding_model
            )

        retriever = index.as_retriever(similarity_top_k=rank)
        nodes = retriever.retrieve(query)

        # print("retrieved nodes:", len(nodes))
        # for i, node in enumerate(nodes):
        #     print(f"[{i}] score={node.score!r}")
        #     try:
        #         print(f"[{i}] text={node.node.get_content()[:200]!r}")
        #     except Exception as e:
        #         print(f"[{i}] text=<error: {e}>")
        #     print(f"[{i}] metadata={getattr(node.node, 'metadata', None)}")

        # スコアしきい値は要調整
        has_good_context = len(nodes) > 0 and any(
            (node.score is not None and node.score > threshold) for node in nodes
        )

        text_qa_template = PromptTemplate(prompt_text)

        if has_good_context:
            print(f"💡 LLM returns good context!!")

            query_engine = index.as_query_engine(
                llm=llm_model,
                similarity_top_k=3,
                text_qa_template=text_qa_template,
            )
            res = query_engine.query(query)
            answer = res.response.strip()
        else:
            prompt = text_qa_template.format(query_str=query, context_str="")
            raw = llm_model.complete(prompt)
            answer = getattr(raw, "text", str(raw)).strip()

        print(f"👤: {query}\n🤖: {answer}")
    finally:
        try:
            if vector_store is not None:
                asyncio.run(vector_store.aclose())
        finally:
            # To prevent close() from being called again in the __del__ method of the llama-index OpenSearch integration, disable internal references.
            # avoid: RuntimeWarning: coroutine 'AsyncOpenSearch.close' was never awaited
            if client is not None:
                if hasattr(client, "_os_async_client"):
                    client._os_async_client = None
                if hasattr(client, "_os_client"):
                    client._os_client = None
