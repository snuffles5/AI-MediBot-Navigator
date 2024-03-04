import logging
import os
import sys
from pathlib import Path

DEFAULT_FOLDER_NAME = "logs"


def setup_logging(log_directory: Path, log_name: str = "log.log"):
    # Create 'logs' directory if it doesn't exist
    if not log_directory.exists():
        os.makedirs(log_directory)

    # Check if running in debug mode
    is_debug_mode = sys.gettrace() is not None

    # Define logging level based on debug mode
    console_logging_level = logging.DEBUG if is_debug_mode else logging.INFO

    # Create logger
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)  # Set logger to the highest level of detail

    # Formatter
    formatter = logging.Formatter(
        "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    )

    # File handler (always logs at INFO level and above)
    file_handler = logging.FileHandler(log_directory / log_name)
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(formatter)

    # Stream handler (console), level depends on debug mode
    stream_handler = logging.StreamHandler()
    stream_handler.setLevel(console_logging_level)
    stream_handler.setFormatter(formatter)

    # Add handlers to logger
    logger.addHandler(file_handler)
    logger.addHandler(stream_handler)

    return logger


# Setup and use logger
logs_directory = Path("../")
logger = setup_logging(logs_directory)

logger.info("Starting Logger")
