//
// Created by Stefan Annell on 2023-10-29.
//

#pragma once
#include "SFML/Graphics.hpp"

class Controls {
public:
    void HandleEvents(sf::RenderWindow &sfmlWin);

    using EventCallback = std::function<void(const sf::Event&)>;
    void RegisterEvent(sf::Event::EventType type, const EventCallback& callback);

private:
    std::map<sf::Event::EventType, EventCallback> EventCallbacks;
};
