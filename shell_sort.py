# shell_sort.py
#
# 2016-03-20
#
# idea after: https://en.wikipedia.org/wiki/Shellsort
# number of comparisons: << n*n
#
#
# PEP8 check (http://pep8online.com): OK
# test on: Win8.1 (64 Bit), CLI Python 3.5.0: OK
# test on http://ideone.com: OK


# !!BEST!! with a "small" array with N = 100
# step distances acc. Marcin Ciura, 2001, empirically derived
# # = 9:
gaps = [1750, 701, 301, 132, 57, 23, 10, 4, 1]
# => n = 348

# !STILL GOOD! for "small" arrays
# original step distances acc. Donald Shell, 1959: 1,2,4,8,16,...,2^n
# # = 9:
# gaps = [256, 128, 64, 32, 16, 8, 4, 2, 1]
# => n = 379

# !!2nd LAST!! => not good for "small" arrays
# step distances acc. Sedgewick, Robert, 1986: "Algorithms in C", 3rd ed.
# 1, 4^k + 3*2^(k-1)+1 for k = 1, 2, 3,...
# # = 9:
# gaps = [65921, 16577, 4193, 1073, 281, 77, 23, 8, 1]
# => n = 416

# OK
# step distances acc. Tokuda, Naoyuki, 1992: "An Improved Shellsort":
# (9^k - 4^k)/(5*4^(k-1))
# # = 9:
# gaps = [1182, 525, 233, 103, 46, 20, 9, 4, 1]
# => n = 387

# !!LAST!! => not good for "small" arrays
# step distances acc. Vaughan Ronald Pratt, 1979:
#  "Shellsort and Sorting Networks"
# (3^k-1)/2, not greater than N/3
# # = 9:
# gaps = [9841, 3280, 1093, 364, 121, 40, 13, 4, 1]
# => n = 425

# !!2nd NARROW BEST!!
# acc. Papernov, A. A., Stasevich, G. V., 1965:
#  "A Method of Information Sorting in Computer Memories"
# 1, 2^k+1
# # = 9:
# gaps = [257, 129, 65, 33, 17, 9, 5, 3, 1]
# => n = 351


# input list; cf. https://www.random.org
x = [
    474,
    378,
    601,
    169,
    358,
    606,
    461,
    522,
    145,
    955,
    985,
    776,
    829,
    793,
    803,
    674,
    596,
    382,
    913,
    642,
    619,
    265,
    450,
    887,
    688,
    200,
    477,
    795,
    560,
    227,
    515,
    310,
    939,
    394,
    673,
    583,
    80,
    813,
    943,
    941,
    782,
    800,
    71,
    959,
    498,
    985,
    965,
    865,
    982,
    547,
    563,
    634,
    601,
    409,
    777,
    165,
    653,
    612,
    299,
    95,
    406,
    219,
    546,
    597,
    656,
    471,
    566,
    797,
    143,
    728,
    287,
    885,
    811,
    900,
    783,
    341,
    713,
    634,
    760,
    80,
    847,
    734,
    994,
    839,
    379,
    695,
    407,
    577,
    415,
    784,
    777,
    589,
    642,
    425,
    401,
    397,
    261,
    687,
    863,
    769
]


print("\nx: ", x)
l = len(x)
print ("\nlength of list: ", l)

for i in x:
    print("\n", i)

# number of comparisons:
n = 0


# sort an array x[0...l-1]
# start with the largest gap and work down to a gap of 1:
for gap in gaps:

    # do a gapped insertion sort for this gap size
    # the first gap elements x[0..gap-1] are already in gapped order
    # keep adding one more element until the entire array is gap sorted:
    i = gap
    while i < l:

        # add x[i] to the elements that have been gap sorted
        # save x[i] in temp and make a hole at position i:
        temp = x[i]

        # shift earlier gap-sorted elements up until
        # the correct location for x[i] is found:
        j = i
        while j >= gap and x[j - gap] > temp:
            x[j] = x[j - gap]
            j -= gap
            n += 1

        # put temp (the original x[i]) in its correct location:
        x[j] = temp

        i += 1


print ("\nresults:")
for i in x:
    print("\n", i)
print ("\nnumber of comparisons: ", n)


# end of shell_sort.py
