import sys
import time
from absl import logging, app, flags
from intrinsic.skills.python import skill_interface

FLAGS = flags.FLAGS
flags.DEFINE_integer("port", 8003, "Port to listen on.", allow_override=True)
flags.DEFINE_string("skill_service_config_filename", "", "Path to config.", allow_override=True)

class TestPythonSkill(skill_interface.Skill):
    def execute(self, request, context):
        logging.info("Executing TestPythonSkill")
        return None

def main(argv):
    logging.info("TestPythonSkill started")
    # Simulate running by sleeping
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logging.info("TestPythonSkill stopped")

if __name__ == '__main__':
    app.run(main)
