# lambda_expression.py
# save a UTF-8 encoded file due to '°' !!
# RSa, 2016-03-27
#
# idea after: QPB, p.111, ch. 9.6
#
#
# PEP8 check (http://pep8online.com): OK
# test on: Win8.1 (64 Bit), CLI Python 3.5.0: OK
# test on http://ideone.com: no std input

t3 = {'FtoC': lambda deg_f: (deg_f - 32) * 5 / 9}

str_input = input('Enter temperature in Fahrenheit: ')

f_input = float(str_input)
if f_input >= -459.00 and f_input < 9941.00:
    print("\ntemperature in Celsius: ", end=' ')

    deg_C = t3['FtoC'](f_input)

    formatting = '{:,.2f}'
    print(formatting.format(deg_C))

else:
    print("\nInput out of range: -459.00 °F <= input < 9941.00 °F")

# end of lambda_expression.py
