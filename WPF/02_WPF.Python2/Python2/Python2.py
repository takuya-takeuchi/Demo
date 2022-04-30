class PythonTest():
    def twice(selt, array):
        list = [0 for i in range(len(array))]
        i = 0
        for x in array:
            list[i] = x * 2
            i += 1
        return list;
    