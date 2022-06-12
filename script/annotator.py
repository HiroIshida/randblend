import os

import matplotlib.image as mpimg
import matplotlib.pyplot as plt

from randblend.path import get_root_path

snapshots_path = get_root_path() / "gso_snapshots"

lst = []

for png_filepath in snapshots_path.iterdir():
    img = mpimg.imread(str(png_filepath))
    plt.imshow(img)
    plt.show()
    name, _ = os.path.splitext(png_filepath.name)
    print(name)
    # name = png_filepath.suffix("").name
    # print(name)

    inp = input()
    if inp == "y":
        print("adding {}".format(name))
        lst.append(name)
