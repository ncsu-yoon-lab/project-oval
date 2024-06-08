from kivy_garden.mapview import MapView, MapMarker
from kivy.network.urlrequest import UrlRequest
from kivy.app import App
from kivy.uix.label import Label
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.relativelayout import RelativeLayout
from kivy.uix.button import Button
from kivy_garden.graph import Graph, MeshLinePlot
from kivy.lang import Builder
from kivy.clock import Clock
from kivy.uix.widget import Widget
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.core.window import Window
from kivy.uix.dropdown import DropDown
from kivy.uix.image import Image
from kivy.uix.spinner import Spinner
from kivy.graphics import *
from kivy.core.window import Window

Builder.load_file('design.kv')

class StartScreen(Screen):

    # Initialize the start screen
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        pass

class ResearchScreen(Screen):

    # Initializing lat, long, and zoom
    lat = long = zoom = 19
    lat = 35.770849
    long = -78.674706

    # Initialize the start screen
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Initialize the mapview
        self.mapview = MapView(zoom = self.zoom, lat = self.lat, lon = self.long)

        # Creating a plot for tracking the data
        self.graphview = Graph(xlabel='X', ylabel='Y', x_ticks_minor=5, x_ticks_major=25, y_ticks_minor=1,
                      y_ticks_major=10, y_grid_label=True, x_grid_label=True, padding=5,
                      xlog=False, ylog=False, xmin=-0, xmax=100, ymin=-1, ymax=1)
        plot = MeshLinePlot(color=[1, 0, 0, 1])
        plot.points = [(x, 0.5 * x * 0.01) for x in range(0, 101)]
        self.graphview.add_plot(plot)


        # Create the research widget that holds either the plot or the map
        self.research_widget = self.ids.research_widget

        # Add the map to the widget
        map = self.ids.research_widget
        map.add_widget(self.mapview)

        # Set the initial switch to 0
        self.widget_switch_state = 0
        self.log_switch_state = 0
        self.route_switch_state = 0
        
    # Allows the user to switch between views of the current tracked positions 
    # and the statistics between each position tracked (Percent error)
    def switch_widgets(self):
        
        # Get the current widget being displayed
        widget = self.research_widget.children[0]

        # Remove that current widget
        self.research_widget.remove_widget(widget)

        # If the switch state is 0, change it to 1 then display the graph
        # Else change it to 1 then display the map
        if self.widget_switch_state == 0:
            self.widget_switch_state += 1
            self.ids.switch.text = 'Display Map'
            self.research_widget.add_widget(self.graphview)
        else:
            self.widget_switch_state -= 1
            self.ids.switch.text = 'Display Statistics'
            self.research_widget.add_widget(self.mapview)

    # Logs the data of the GPS signals received via communication with ESP
    def log_data(self):

        # If the log switch state is 0, the data will be logged consistently to a local csv file
        # Else the logging of the data is stopped
        if self.log_switch_state == 0:
            pass
        else:
            pass

    # Generate a constant set of waypoints for the car to travel and be displayed on the map
    def generate_loop(self):
        pass

class PrototypeScreen(Screen):

    # Initialize the start screen
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        pass

class WW_MonitoringApp(App):

    def build(self):
        Window.fullscreen = 'auto'
        sm = ScreenManager()
        Window.clearcolor = (230/255, 230/255, 230/255, 1)
        sm.add_widget(StartScreen(name='start_screen'))
        sm.add_widget(ResearchScreen(name='research_screen'))
        sm.add_widget(PrototypeScreen(name='prototype_screen'))

        return sm

if __name__ == '__main__':
    WW_MonitoringApp().run()
