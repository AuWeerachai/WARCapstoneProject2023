
def test_init():
    '''
    Tests the window init function
    >>> test_init()
    '''
    return window.init()

if __name__ == '__main__':
    import window
    from doctest import testmod
    testmod(verbose=True)