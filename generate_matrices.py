from numpy import random, matmul, array, reshape
import sys


def main():
    a_rows = int(sys.argv[1])
    a_columns = int(sys.argv[2])

    b_rows = int(sys.argv[3])
    b_columns = int(sys.argv[4])

    a_ints = random.randint(10000, size=(a_rows*a_columns))
    b_ints = random.randint(10000, size=(b_rows*b_columns))

    a_floats = random.rand(a_rows*a_columns)
    b_floats = random.rand(b_rows*b_columns)

    a = []
    b = []

    for i in range(0, len(a_floats)):
        a.append(float(a_ints[i]) + a_floats[i])

    for i in range(0, len(b_floats)):
        b.append(float(b_ints[i]) + b_floats[i])

    print_matrix(a)

    print_matrix(b)

    mat_a = array(a)
    mat_b = array(b)

    mat_a = reshape(mat_a, (-1, a_columns))
    mat_b = reshape(mat_b, (-1, b_columns))

    result = matmul(mat_a, mat_b)

    print(result)



def print_matrix(matrix):
    print("[", end="")
    for i, num in enumerate(matrix):
        if i == len(matrix) - 1:
            print(str(num) + "]")
        else:
            print(str(num) + ", ", end="")


if __name__ == "__main__":
    main()
