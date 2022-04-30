import simplejson

class PythonTest():
    def load(self, path):
        f = open(path, 'r')
        data = json.load(f)
        f.close()
        return data
    