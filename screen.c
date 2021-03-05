// This module implements the Z-100 screen using the gtk graphics library
#include <stdio.h>
#include <stdlib.h>
#include <gtk/gtk.h>
#include "video.h"
#include "screen.h"
#include "keyboard.h"
#include "mainBoard.h"

// get access to external types define elsewhere
// **DO WE NEED THESE?? SHOULD THEY NOT COME FROM THE MAINBOARD DEFINITIONS via
// A HEADER FILE?
extern Video* video;
extern keyboard* keybrd;
//extern insigned int* pixels;

//void renderScreen(Video*, unsigned int*);

const int X_SCALE = 2;
const int Y_SCALE = 4;

GtkWidget *window;
GtkWidget *drawingArea;

// gtk function to draw in gtk window
void display() {
  gtk_widget_queue_draw(window);
}

static gboolean on_keypress(GtkWidget* widget, GdkEventKey* event) {
  // the 'return' char will be used for any incoming keycode that does not
  // correspond to a standard ascii digit or letter - (set the default key code
  // to '\r')
  char code = '\r';
  /* the GdkEventKey* event is a keyboard hardware event that is detected
    from the user machine. What gets used here is the event->keyval instance
    variable of the GdkEventKey object.
    "keyval" is either one of two things - an ascii character or a keycode */
  printf("*key pressed* GdkEventKey Code: %x\n", event->keyval);
  /* for now, only process letters and digits
    - if event->keyval is a standard ascii digit or letter (0-Z) */
  if(event->keyval >= '0' && event->keyval <= 'z') {
    // change the code used to the corresponding ascii code
    code = event->keyval;
    // otherwise, code will be a RETURN (0x0D) code
  }
  // call function keyaction in keyboard.c, which loads the keyboard buffer
  // with the pressed key code
  // ** NOTE: event->keyval MAY NOT MATCH THE Z-100 KEY CODES - FIX THIS!! **
  keyaction(keybrd, code);
}

static gboolean on_draw_event(GtkWidget* widget, cairo_t *cairo_obj) {
  // holds 24-bit color from pixel array element
  unsigned int p24BitColor;
  // chars to hold each RGB color value
  unsigned char red_val;
  unsigned char green_val;
  unsigned char blue_val;
  /* populate the pixel array using the render screen function defined in video.c.
    This function reads the VRAM and sets up the pixel array accordingly */
  renderScreen(video, pixels);
  // now, cycle through the pixel array as if reading rows/columns
  // loop through rows
  for(int row = 0; row < 225; row++) {
    // loop through columns
    for(int column = 0; column < 640; column++) {
      // get 24-bit colour from pixel array element
      p24BitColor = pixels[(row*640) + column];
      // extract each color component from the 24-bit color
      red_val = (p24BitColor>>16)&0xff;
      green_val = (p24BitColor>>8)&0xff;
      blue_val = p24BitColor&0xff;
      // source RGB data to cairo rectangle
      cairo_set_source_rgb(cairo_obj, red_val, green_val, blue_val);
      // make rectangle (obj, x-coor of left side, y-coor of top, width, height)
      cairo_rectangle(cairo_obj, column*X_SCALE, row*Y_SCALE, X_SCALE, Y_SCALE);
      // color rectangle
      cairo_fill(cairo_obj);
    }
  }
  return FALSE;
}

void screenInit(int* argc, char** argv[]) {

  gtk_init(argc, argv);

  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  drawingArea = gtk_drawing_area_new();
  // add drawing area to window
  gtk_container_add(GTK_CONTAINER(window), drawingArea);

  gtk_window_set_title(GTK_WINDOW(window), "Z-100 Screen");
  gtk_window_set_default_size(GTK_WINDOW(window), 640*X_SCALE, 225*Y_SCALE);

  // connect callback functions to GTK window and drawing area default operations
  /* i.e. - when gtk_widget_queue_draw() is called via the display() function called
    from the main Z-100 loop in mainBoard.c, it qualifies as a "draw" event */
  g_signal_connect(G_OBJECT(drawingArea), "draw", G_CALLBACK(on_draw_event), NULL);
  g_signal_connect(G_OBJECT(drawingArea), "destroy", G_CALLBACK(gtk_main_quit), NULL);
  g_signal_connect(G_OBJECT(window), "key-press-event", G_CALLBACK(on_keypress), NULL);

  gtk_widget_show_all(window);
}

void screenLoop() {
  // keep GTK window open - will not return
  gtk_main();
}
