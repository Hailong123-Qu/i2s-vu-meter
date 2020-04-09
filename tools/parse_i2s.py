import matplotlib.pyplot as plt

f_lst = []

with open('tools/sample_output.log', 'r') as f:
    t = f.read()

    for i in range(len(t)):
        if t[i] == 'R' or t[i] == 'L':
            f_lst.append((t[i], t[i+1:i+11]))

l_lst_str = []
r_lst_str = []

for val in f_lst:
    if val[0] == 'L':
        l_lst_str.append(val[1])
    else:
        r_lst_str.append(val[1])
        
l_lst = [int(x) for x in l_lst_str]
r_lst = [int(x) for x in r_lst_str]
