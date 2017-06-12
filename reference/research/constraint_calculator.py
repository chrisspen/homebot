import argparse
import os
import sys

from pyexcel_ods import ODSBook

class ConstrainableMixin(object):

    @classmethod
    def constrain(cls, **kwargs):
        return ConstrainedComponent(cls, kwargs)

class ConstrainedComponent(ConstrainableMixin):
    """
    Represents a component that has had its resources constrained in some way.
    """

    def __init__(self, cls, constraints):
        self.cls = cls
        self.constraints = dict(constraints)

    def __getattr__(self, name):
        if name in ('cls', 'constraints'):
            return super(ConstrainedComponent, self).__getattribute__(name)
        cls = super(ConstrainedComponent, self).__getattribute__('cls')
        return getattr(cls, name)

class Component(ConstrainableMixin):

    datasource = None

    def __init__(self):
        pass

    def initAttributes(self):
        pass

class GroupComponent(Component):

    datasource = None

class System(Component):

    def __init__(self):
        pass

    def print_list(self, name=None, attribute=None):
        d = type(self).__dict__.copy()
        d.update(self.__dict__)
        for _k, _v in d.iteritems():
            if not isinstance(_v, Resource):
                continue
            if name and _k != name:
                continue
            print _k, _v

            if attribute:
                if attribute == 'data':
                    if _v.type.datasource:
                        #print '\t',_v.type.datasource
                        assert os.path.isfile(_v.type.datasource)
                        book = ODSBook(_v.type.datasource)
                        sheet = book.sheets()['Sheet1']
                        headers = sheet[0]
                        data = sheet[1:]
                        for row in data:
                            print dict(zip(headers, row))
                    else:
                        print '\t',_v.type.datasource

class Resource(object):

    def __init__(self, type, units=None, default=None):
        self.type = type
        self.units = units
        self.default = default

if __name__ == '__main__':
    pass
