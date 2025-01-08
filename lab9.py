import numpy as np


def even(arr):
    arr = np.array(arr)  # создаём массив
    unique, counts = np.unique(arr, return_counts=True)  # находим уникальные элементы массива и их количество
    need_nums = unique[counts % 2 == 0]  # определяем, какие элементы встречаются чётное количество раз
    res = arr[np.isin(arr, need_nums)]  # сохраняем те, которые встречаются чётное количество раз
    return res  # возвращаем массив


mas = input("Элементы массива через пробел: ")
mas = list(map(int, mas.split()))

print(even(mas))
