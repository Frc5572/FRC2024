import os

TUPLES_JAVADOC = """
{0}/**
{0}* Defines various tuple types for Java.
{0}*/
"""

IVALUE_JAVADOC = """
{0}/**
{0}* A type that has a {1} element.
{0}*/
"""

IVALUE_GETTER_JAVADOC = """
{0}/**
{0}* Get the {1} element.
{0}*/
"""


TUPLE_JAVADOC = """
{0}/**
{0}* A tuple of {1} elements.
{0}*/
"""

TUPLE_CONSTRUCTOR_JAVADOC = """
{0}/**
{0}* Create a new {1}-tuple.
{0}*/
"""

def gentuples():
    with open(os.path.join(os.path.dirname(__file__), 'Tuples.java'), 'w') as f:
        f.write('package frc.lib.types;\n\n{}public final class Tuples {{\n    private Tuples() {{}}\n\n'.format(TUPLES_JAVADOC.format('')))
        for tup in range(1,10):
            nth = "{}th".format(tup)
            if(tup % 10 == 1 and tup != 11):
                nth = "{}st".format(tup)
            if(tup % 10 == 2 and tup != 12):
                nth = "{}{}{}".format(tup, "n", "d")
            if(tup % 10 == 3 and tup != 13):
                nth = "{}rd".format(tup)
            # IValue interface
            f.write(IVALUE_JAVADOC.format('    ', nth))
            f.write('    public static interface IValue{}<T> {{\n'.format(tup))
            f.write(IVALUE_GETTER_JAVADOC.format('        ', nth))
            f.write('        T _{}();\n'.format(tup - 1))
            f.write('    }\n\n')
            # Tuple Class
            f.write(TUPLE_JAVADOC.format('    ', tup))
            f.write('    public static final class Tuple{}<'.format(tup))
            for t in range(1, tup):
                f.write('T{}, '.format(t))
            f.write('T{}> implements'.format(tup))
            for t in range(1, tup):
                f.write(' IValue{}<T{}>,'.format(t, t))
            f.write(' IValue{}<T{}> {{\n'.format(tup, tup))
            # Fields
            for t in range(1, tup + 1):
                f.write('        private final T{} v{};\n'.format(t, t - 1))
            # Constructor
            f.write(TUPLE_CONSTRUCTOR_JAVADOC.format('        ', tup))
            f.write('        public Tuple{}('.format(tup))
            for t in range(1, tup):
                f.write('T{} v{}, '.format(t, t - 1))
            f.write('T{} v{}) {{\n'.format(tup, tup - 1))
            for t in range(1, tup + 1):
                f.write('            this.v{} = v{};\n'.format(t - 1, t - 1))
            f.write('        }\n\n')
            # Accessors
            for t in range(1, tup + 1):
                f.write('        @Override\n        public T{} _{}() {{\n            return this.v{};\n        }}\n\n'.format(t, t-1, t-1))
                pass
            f.write('    }\n\n')
            pass
        f.write('}\n')
        pass
    pass

if __name__ == '__main__':
    gentuples()
