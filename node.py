class Node:
    def __init__(self, mid):
        self.left = None
        self.right = None
        self.val = mid

    @staticmethod
    def in_order(root):
        res = []
        if root:
            res = root.in_order(root.left)
            res.append(root.val)
            res = res + root.in_order(root.right)
        return res

