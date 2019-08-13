from enum import Enum


class IssueType(Enum):
    """
    A list of issue types.
    """
    def __str__(self):
        return self.name

    # Lane Issues
    MULTIPLE_TRANSITION_PROPERTIES = 0
    NON_EXISTANT_JUNCTION_REF = 1

    # Junction Issues
    MULTIPLE_INFLOW_OUTFLOWS = 2
    POST_TRANSITION_MULTIPLE_NORMAL = 3
    POST_TRANSITION_NO_SPLIT = 4
    POST_TRANSITION_NO_NORMAL = 5
    PRE_TRANSITION_MULTIPLE_NORMAL = 6
    PRE_TRANSITION_NO_MERGE = 7
    PRE_TRANSITION_NO_NORMAL = 8
