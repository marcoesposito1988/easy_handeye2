import sys

from rqt_gui.main import Main
from easy_handeye2.handeye_rqt_evaluator import RqtHandeyeEvaluator


def main(argv=sys.argv):
    plugin = 'easy_handeye2.handeye_rqt_evaluator.RqtHandeyeEvaluator'
    main = Main(filename=plugin)
    sys.exit(main.main(argv,
                       standalone=plugin,
                       plugin_argument_provider=RqtHandeyeEvaluator.add_arguments))


if __name__ == '__main__':
    main()
