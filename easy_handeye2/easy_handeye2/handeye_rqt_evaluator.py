from rqt_gui_py.plugin import Plugin
from rqt_py_common.plugin_container_widget import PluginContainerWidget
from easy_handeye2.handeye_rqt_evaluator_widget import RqtHandeyeEvaluatorWidget


class RqtHandeyeEvaluator(Plugin):
    def __init__(self, context):
        super(RqtHandeyeEvaluator, self).__init__(context)
        self.setObjectName('RqtEasyHandEyeEvaluator')

        self._plugin_widget = RqtHandeyeEvaluatorWidget(context)
        self._widget = PluginContainerWidget(self._plugin_widget)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)

    @staticmethod
    def add_arguments(parser):
        pass
