# Planning Graph Node

The planning graph node is used as a research vessel for automated plan step generation based on experience.

## Running it

To use the node, simply start it via `rosrun`:

```bash
$ rosrun planning_graph_node server.py
```

To test its capabilities, send one of these sample queries and it will
return a sample result:

```bash
$ rosservice call /planning_server/plan "pattern: 'fetch ?object from ?location' \
  bindings: \
  - key: '?location' \
    value: 'table'"

$ rosservice call /planning_server/plan "pattern: 'fetch ?object from ?location' \
  bindings: \
  - key: '?object' \
    value: 'cheese'"

$ rosservice call /planning_server/plan "pattern: 'fetch ?object from ?location' \
  bindings: \
  - key: '?object' \
    value: 'cheese' \
  - key: '?object' \
    value: 'tomato' \
  - key: '?location' \
    value: 'fridge'"
```


## What it does

The node will resolve all possible combinations of the given
`bindings` within the supplied `pattern` using `?` keywords (this is
borrowed from Prolog). If one or more of the keywords are omitted, default values are assumes.


## Alpha Phase Notice

This software is in its very early alpha phase and doesn't do much
right now beyond serving as a development vessel. It will become more
useful when the planned features are implemented.
