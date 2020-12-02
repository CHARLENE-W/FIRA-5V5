# DLLStrategy

Template project for DLL strategy.

## Description

This template is for the [DLLAdapter](https://github.com/npuv5pp/V5DLLAdapter) project of the Simuro5v5 platform.

The new interface functions include:

- `void GetTeamInfo(TeamInfo* teaminfo)`

  Used to specify strategy information, currently contains the team name field.

  The parameter `TeamInfo* teaminfo`** requires the strategy to populate its own information**, which will be returned to the platform.

- `void GetInstruction(Field* field)`

  Each beat in the game is called, ** requires a strategy to specify the wheel speed**, which is equivalent to the strategy of the old interface.

  The parameter `Field* field` is the `In/Out` parameter, which stores the current field information and allows the strategy to modify its own wheel speed.

- `void GetPlacement(Field* field)`

  Called every time the auto-position is placed, ** requires a strategy to specify the placement information**.

  The parameter `Field* field` is the `In/Out` parameter, which stores the current field information and allows the strategy to modify its position (and the position of the ball).

- `void OnEvent(EventType type, void* argument)`

  Called when an event occurs.

  The parameter `EventType type` indicates the event type;

  The parameter `void* argument` indicates the argument to the event, or NULL if there is no argument.

## Other

The new match interface function accepts the parameter type `Field`, which contains the field information, similar to the `Environment` in the old interface, but `gamestate` and `whosball` are not saved in `field`. Because these variables only change at the beginning of the round and this information can be obtained from the `OnEvent` function.

## Credits

Copyright (C) Northwestern Polytechnical University V5++ team. All rights reserved.