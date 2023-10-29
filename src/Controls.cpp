//
// Created by Stefan Annell on 2023-10-29.
//

#include "Controls.h"
void Controls::HandleEvents(sf::RenderWindow &sfmlWin) {
    sf::Event e;
    while (sfmlWin.pollEvent(e)) {
        if (EventCallbacks.contains(e.type)) {
            EventCallbacks[e.type](e);
        }
    }
}

void Controls::RegisterEvent(sf::Event::EventType type, const EventCallback& callback) {
    EventCallbacks[type] = callback;
}
