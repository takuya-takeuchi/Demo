def get_labels(label_path: str):
    label = list()
    with open(label_path, mode='rt', encoding='utf-8') as f:
        for line in f.readlines():
            line = line.strip()
            if len(line) == 0:
                continue
            label.append(line)
    
    return label,
