# file_names_to_data.py
"""
This module plots gas prices which have been retrieved from
filenames of photos with prices at gas stations
"""
#
# 2021-05-30
#
# Python 3.7.9 (tags/v3.7.9:13c94747c7, Aug 17 2020, 18:58:18)
# [MSC v.1900 64 bit (AMD64)]
#
#
# test: OK
#
#
# to do:
#
#
# PEP8 check (http://pep8online.com): OK

import os  # use os methods
import datetime
import re  # regular expressions

import matplotlib.pyplot as plt
# import matplotlib.ticker as mtick

import pandas as pd


workdir = "C:\\temp\\Super"
TITLE = "Super gas prices [EUR per liter] in Munich area (Germany)\
\narbitrary spot checks (no duplicate days)"

os.chdir(workdir)

gas_prices = []  # list with gas prices in [EUR]


for folderName, subfolders, filenames in os.walk(workdir):
    for filename in filenames:
        # find all photos with gas prices:
        re1 = re.match("Super*", filename)
        if re1:  # if True
            gas_prices.append(filename)


n1 = len(gas_prices)
photo_dates = []
photo_prices = []

if n1 > 0:
    print(n1, "filenames with gas prices found!")

    # get date and gas price from each filename:
    for filename in gas_prices:
        # get date from filename:
        modification_time = os.path.getmtime(filename)
        photo_time = datetime.datetime.fromtimestamp(modification_time)
        photo_dates.append(photo_time.date())

        # get price from filename:
        re2 = re.search("\d+,\d+", filename)
        photo_prices.append(re2.group())
        # group(): extract the matching string

        # have correct number format: , --> .
        photo_prices = [str1.replace(",", '.') for str1 in photo_prices]


else:
    print("No filenames with gas prices found!")


##################################
#
# pandas data clean-up:
#  - data grooming: there can be more than one price per day
#    => take only the last price
#
# create a DataFrame:
df1 = pd.DataFrame({"price": photo_prices}, index=photo_dates)

# eliminate duplicates:
df1a = df1[~df1.index.duplicated(keep='last')]

# type = str !!!
df1a = df1a.astype(float)


##################################
#
# plotting:
fig, ax = plt.subplots(figsize=(16, 9))

ax.plot(df1a.index, df1a.price)

# have a plot title:
plt.title(TITLE, fontsize=20)

plt.xticks(fontsize=14)
plt.yticks(fontsize=14)

# turn a grid on:
plt.grid()

plt.show()


# end of file_names_to_data.py
