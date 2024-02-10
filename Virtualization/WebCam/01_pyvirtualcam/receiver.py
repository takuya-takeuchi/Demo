import argparse
import os
import sys

import cv2
import logging
from logging import config
import yaml

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=int)
    return parser.parse_args()

def get_logger(logger_name: str):
    os.makedirs("logs", exist_ok=True)
    with open("./logging.yaml", encoding='utf-8') as f:
        logconfig = yaml.safe_load(f)
        config.dictConfig(logconfig)
    return logging.getLogger(logger_name)

def receive_frame(source: int, logger: logging.Logger):
    cap = cv2.VideoCapture(source)

    logger.info("press enter key to stop program")
    while True:
        ret, frame = cap.read()
        if ret:
            cv2.imshow("Window Name", frame)
            
        if cv2.waitKey(30) == 13:
            break

    cv2.destroyAllWindows()
    cap.release()

if __name__ == '__main__':
    # setup logger
    logger = get_logger("receiver")

    # parse args
    args = get_args()
    source = args.source

    logger.info("Arguments")
    logger.info("\tsource: {}".format(source))

    receive_frame(source, logger)