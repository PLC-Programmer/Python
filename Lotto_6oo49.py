# Lotto_6oo49.py
# 
# 2015-10-21
#
# test on: Win8.1 (64 Bit), CLI Python 3.5.0: OK
# PEP8 check (http://pep8online.com): OK

import random  # use random function
from datetime import date  # get date objects


# endless loop to be broken by the user
j = 1  # the number of my drawing
while True:

    # store result in a list for later printing in a line:
    rand_list = []

    # generate 6 sampled, integer random numbers out of 49 and return a list:
    # ATTENTION: range(1,49) will not get you integer number 49!
    rand_list = random.sample(range(1, 50), 6)
    # a while loop over random.randint(1, 49) returns a random integer
    # such that a <= N <= b:
    # --> one problem: this creates duplicative numbers!
    # => take sample method instead w/o while loop
    # with xrange function to generate a list of integers from 1 to 49

    # print(rand_list)

    # sort list in place:
    rand_list.sort()

    # build a line with date and numbers which can be saved to disk:
    # get todays date in ISO format: YYYY-MM-DD
    d = date.today()
    today = d.isoformat()
    line = "\n" + today + ": my draw no. " + str(j) + ": "

    for i in rand_list:
        # format an individual number before adding to line:
        no_str = "{: > 4d}".format(i)
        print(no_str, end=' ')
        line += " "
        line += no_str
    # line += "\n"
    print()

    # ask user for disk storage:
    answer = input(
        'Save numbers to disk? [y=save & next round / n=no save & exit] ')
    if answer == 'y':
        print(line)
        # print(answer)
        f = open("Lotto_6oo49.txt", "a")
        f.write(line)
        f.close

        print()
        j += 1
    else:
        # add a new line to end of file:
        f = open("Lotto_6oo49.txt", "a")
        f.write("\n")
        f.close

        print("bye.")
        break


# output file:
# 2015-10-21: my draw no. 1:     7   13   24   26   38   47
# 2015-10-21: my draw no. 2:    13   20   22   24   35   40
# 2015-10-21: my draw no. 3:     1    4    5   18   27   37

# end of Lotto_6oo49.py --> PEP8 doesn't like that!
