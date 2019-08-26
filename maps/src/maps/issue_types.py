from enum import Enum


class IssueType(Enum):
    """
    A list of issue types.
    """
    def __str__(self):
        return self.name

    # Lane Issues
    NON_EXISTANT_JUNCTION_REF = 1
    INVALID_DIRECTION_OF_TRAVEL = 12

    # Junction Issues
    NO_OUTFLOW = 2
    SPLIT_MULTIPLE_NORMAL = 3
    SPLIT_NO_SPLIT_PROPERTY = 4
    SPLIT_NO_NORMAL = 5
    MERGE_MULTIPLE_NORMAL = 6
    MERGE_NO_MERGE_PROPERTY = 7
    MERGE_NO_NORMAL = 8
    FROM_SPLIT_MISSING_PROPERTY = 9
    MERGING_MISSING_PROPERTY = 10
    JUNCTION_TOO_FAR = 11
