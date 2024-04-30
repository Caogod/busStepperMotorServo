import logging


def set_log(handle_name, log_file):
    logger = logging.getLogger(handle_name)
    logger.setLevel(logging.INFO)
    handler = logging.FileHandler(log_file)
    formatter = logging.Formatter(
        "%(asctime)s %(name)s %(levelname)s %(message)s"
    )
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    return logger

def record_command(self, func):
    def wrapper(*args, **kwargs):
        self._log.info(func.name)
        return func(*args, **kwargs)
    return wrapper