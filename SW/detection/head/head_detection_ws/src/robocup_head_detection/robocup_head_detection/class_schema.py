"""Head object IDs agreed on 2026-10-12, with the original checkpoint adapter."""

CLASSES = ("green_apple", "peach", "tomato", "fanta_can", "cup", "banana")
LEGACY_CLASSES = ("apple", "banana", "fanta_can", "green_apple", "mug", "peach", "plate")
LEGACY_TO_CURRENT = {0: 2, 1: 5, 2: 3, 3: 0, 4: 4, 5: 1, 6: None}


def class_mapping(names):
    """Only adapt this project's exact legacy schema; preserve unrelated models.

    In particular, COCO's apple is an apple and must never become a tomato.
    None means excluded from the current Head detection task.
    """
    ordered = tuple(names[i] for i in range(len(names)))
    if ordered == LEGACY_CLASSES:
        return {old: None if new is None else (new, CLASSES[new])
                for old, new in LEGACY_TO_CURRENT.items()}
    return {i: (i, str(name)) for i, name in enumerate(ordered)}
