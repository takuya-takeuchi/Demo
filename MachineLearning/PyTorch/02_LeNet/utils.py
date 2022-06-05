import os
from logging import config, getLogger
import yaml

def get_labels(label_path: str):
    label = list()
    with open(label_path, mode='rt', encoding='utf-8') as f:
        for line in f.readlines():
            line = line.strip()
            if len(line) == 0:
                continue
            label.append(line)
    
    return label

def get_logger(logger_name: str):
    os.makedirs("logs", exist_ok=True)
    with open("./logging.yaml", encoding='utf-8') as f:
        logconfig = yaml.safe_load(f)
        config.dictConfig(logconfig)
    return getLogger(logger_name)
