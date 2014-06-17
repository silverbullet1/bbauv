#!/usr/bin/env python
import timeit

def main():
    t = timeit.Timer("test.music_3d(complexList)", "import test")
    t.timeit()

if __name__ == "__main__":
    main()
