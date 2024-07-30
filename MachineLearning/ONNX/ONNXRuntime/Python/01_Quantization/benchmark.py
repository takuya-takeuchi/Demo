import argparse
import numpy as np
import onnxruntime
import time
from tqdm import tqdm
import time

def benchmark(model_path, runs):
    session = onnxruntime.InferenceSession(model_path)
    input_name = session.get_inputs()[0].name

    total = 0.0
    input_data = np.zeros((1, 3, 224, 224), np.float32)
    # Warming up
    _ = session.run([], {input_name: input_data})
    for i in tqdm(range(runs)):
        start = time.perf_counter()
        _ = session.run([], {input_name: input_data})
        end = (time.perf_counter() - start) * 1000
        total += end
    total /= runs
    print(f"Avg: {total:.2f}ms")

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_model", required=True, help="input model")
    args = parser.parse_args()
    return args

def main():
    args = get_args()
    input_model_path = args.input_model

    print("benchmarking model...")
    benchmark(input_model_path, 1000)

if __name__ == "__main__":
    main()
