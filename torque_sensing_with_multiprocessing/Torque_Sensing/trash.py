time = [0]

def my_fun(time):
    i = 0
    while i < 10:
        i += 1
        time.append(i)

my_fun(time)

print(time)