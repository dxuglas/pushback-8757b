#ifndef AUTONOMOUS_SELECTOR_H
#define AUTONOMOUS_SELECTOR_H

// Initialize the autonomous selector UI
void initialize_autonomous_selector();

// Get the selected autonomous routine (0-3)
int get_selected_auton();

// Check if red alliance is selected (true = red, false = blue)
bool is_red_alliance_selected();

#endif // AUTONOMOUS_SELECTOR_H