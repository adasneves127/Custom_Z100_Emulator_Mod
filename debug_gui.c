#include <gtk/gtk.h>
#include "8085.h"
#include "utility_functions.h"
#include "debug_gui.h"

extern P8085 p8085;

GtkWidget *main_window;
GtkWidget *fixed_grid;
GtkWidget *title_8085_label;
GtkWidget *reg_B_8085_label;
GtkWidget *reg_B_8085_value;
GtkWidget *step_button;
GtkWidget *reg_B_8085_value_0;
GtkWidget *reg_B_8085_frame_value;
GtkBuilder *builder;

void on_step_button_clicked(GtkButton*);

void* run_debug_window() {
  // these will be stand in values for the command line arguments
  int argc = 0;
  // initialize GTK using stand in command line arguments
  gtk_init(&argc, NULL);
  // establish connection with xml code to build interface
  builder = gtk_builder_new_from_file("debug_gui_components.glade");
  // build window from xml
  main_window = GTK_WIDGET(gtk_builder_get_object(builder, "main_window"));
  /* close window and quit GUI with X in window title bar - WE DO Not WANT THIS
  because we do not want the window closed before the z100 emulator is done */
  g_signal_connect(main_window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

  // widget refereces
  fixed_grid = GTK_WIDGET(gtk_builder_get_object(builder, "fixed_window_grid"));
  title_8085_label = GTK_WIDGET(gtk_builder_get_object(builder, "title_label_8085"));
  reg_B_8085_label = GTK_WIDGET(gtk_builder_get_object(builder, "reg_B_8085_label"));
  reg_B_8085_value = GTK_WIDGET(gtk_builder_get_object(builder, "reg_B_8085_val"));
  reg_B_8085_value_0 = GTK_WIDGET(gtk_builder_get_object(builder, "reg_B_8085_value_0"));
  reg_B_8085_frame_value = GTK_WIDGET(gtk_builder_get_object(builder, "reg_B_Frame_content_label"));
  step_button = GTK_WIDGET(gtk_builder_get_object(builder, "step_button"));

  // g_signal_connect(step_button, NULL, G_CALLBACK(on_step_button_clicked), NULL);

  gtk_widget_show(main_window);

  gtk_main();
}

void on_step_button_clicked(GtkButton *b) {
  //DEBUG
  puts("updating gui values...\n");
  char test_text[8];
  // from "utility_functions.h"
  intToHexString(p8085.B, test_text);
  printf("%s\n", "TEST");
  // GtkLabel *test_cast = GTK_LABEL(reg_B_8085_value_0);
  gtk_label_set_text(GTK_LABEL(reg_B_8085_value_0), "TEST");
}

// void update_debug_gui_values() {
//   //DEBUG
//   // printf("%s\n", "updating gui values...");
//   char test_text[8];
//   intToHexString(p8085.B, test_text);
//   printf("%s\n", test_text);
//   // GtkLabel *test_cast = GTK_LABEL(reg_B_8085_value_0);
//   // gtk_label_set_text(test_cast, test_text);
// }



// static void on_activate(GtkApplication* app) {
//   GtkWidget *window;
//   window = gtk_application_window_new (app);
//   gtk_window_set_title (GTK_WINDOW (window), "Z100 DEBUG");
//   gtk_window_set_default_size (GTK_WINDOW (window), 800, 800);
//   gtk_widget_show_all (window);
// }
//
// void* run_debug_window() {
//
//
//
//
//   // Create a new application
//   GtkApplication *app = gtk_application_new ("com.github.jmatta697.jz100",
//     G_APPLICATION_FLAGS_NONE);
//   g_signal_connect (app, "activate", G_CALLBACK (on_activate), NULL);
//   g_application_run (G_APPLICATION (app), 0, NULL);
// }
