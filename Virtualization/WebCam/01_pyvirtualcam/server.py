import argparse
import os
import sys

import cv2
import logging
from logging import config
import pathlib
import pyvirtualcam
import yaml

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=str)
    parser.add_argument('--loop', action='store_true')
    return parser.parse_args()

def get_logger(logger_name: str):
    os.makedirs("logs", exist_ok=True)
    with open("./logging.yaml", encoding='utf-8') as f:
        logconfig = yaml.safe_load(f)
        config.dictConfig(logconfig)
    return logging.getLogger(logger_name)

def send_frame(source: str, loop: bool, logger: logging.Logger):
    use_movie = False
    # read image files from directory or movie file
    if os.path.isdir(source):
        directory_path = pathlib.Path(source)
        extensions = ['.jpg', '.jpeg', '.png']
        files_list = [str(file) for file in directory_path.glob('**/*') if file.suffix.lower() in extensions]
        files_list.sort()

        # get width and height from image found at first
        index = 0
        for file in files_list:
            frame = cv2.imread(file)
            index += 1
            if not frame is None:
                logger.info(f"read {file}")
                width=frame.shape[1]
                height=frame.shape[0]
                break
    else:
        use_movie = True
        cap = cv2.VideoCapture(source)
        
        # get width and height from valid frame
        _, frame = cap.read()
        if not frame is None:
            width=frame.shape[1]
            height=frame.shape[0]

    try:
        frame
        width
        height
    except NameError:
        logger.error(f"{source} has no valid images or frames")
        sys.exit()

    # you probably should remove `delay` argument due to runtime error `RuntimeError: 'obs' backend: __init__(): incompatible constructor arguments. The following argument types are supported:`
    with pyvirtualcam.Camera(width=width, height=height, fps=30) as camera:
        logger.info(f'Using virtual camera: {camera.device}')
        while True:
            # conver colorspace and sort order of channels to RGB
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # send frame data to virtual camera
            logger.info("sending frame")
            camera.send(frame)

            # wait until next frame come
            camera.sleep_until_next_frame()

            # get next frame
            if use_movie:
                logger.info(f"read next movie frame")
                ret, frame = cap.read()
                if not ret:
                    logger.info(f"reached to end of frame")
                    if not loop:
                        break
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ret, frame = cap.read()
            else:
                for i in range(index, len(files_list)):
                    frame = cv2.imread(files_list[i])
                    index += 1
                    if not frame is None:
                        logger.info(f"read {files_list[i]}")
                        width=frame.shape[1]
                        height=frame.shape[0]
                        break
                # reached to end of list
                if index >= len(files_list):
                    logger.info(f"reached to end of file list")
                    if not loop:
                        break
                    index = 0

    logger.info(f"finish")
    if use_movie:
        cap.release()

if __name__ == '__main__':
    # setup logger
    logger = get_logger("server")

    # parse args
    args = get_args()
    source = args.source
    loop   = args.loop

    logger.info("Arguments")
    logger.info("\tsource: {}".format(source))
    logger.info("\t  loop: {}".format(loop))

    if not os.path.exists(source):
        logger.error(f"{source} is missing")
        sys.exit()

    send_frame(source, loop, logger)