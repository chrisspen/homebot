#!/usr/bin/env python
import pgi
pgi.install_as_gi()
#from gi.repository import Gtk, GObject
from gi.repository import Gtk

class UI:

    def __init__(self):
        builder = Gtk.Builder()
        builder.add_from_file("gui.glade")

        window = builder.get_object('window1')
        window.connect("delete-event", Gtk.main_quit)

        self.led_button = builder.get_object('togglebutton1')
        self.led_button.connect('toggled', self.on_led_button_toggle)

        pan_adj = Gtk.Adjustment(value=0, lower=0, upper=360+1, step_increment=1, page_increment=1, page_size=1)
        spinBtn = builder.get_object("spinbutton1")
        spinBtn.configure(pan_adj, 1, 0)

        tilt_adj = Gtk.Adjustment(value=0, lower=-30, upper=30+1, step_increment=1, page_increment=1, page_size=1)
        spinBtn = builder.get_object("spinbutton2")
        spinBtn.configure(tilt_adj, 1, 0)

        window.show_all()

    def on_led_button_toggle(self, button):
        if button.get_active():
            state = ['1', 'on']
            button.set_label(state[1].upper())
            #self.send_command(state[0])
        else:
            state = ['0', 'off']
            button.set_label(state[1].upper())
            #self.send_command(state[0])

UI()

# win = Gtk.Window()
# win.connect("delete-event", Gtk.main_quit)
# win.show_all()
Gtk.main()
