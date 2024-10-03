package frc.lib.types;


/**
 * Defines various tuple types for Java.
 */
public final class Tuples {
    private Tuples() {}


    /**
     * A type that has a 1st element.
     */
    public static interface IValue1<T> {

        /**
         * Get the 1st element.
         */
        T _0();
    }


    /**
     * A tuple of 1 elements.
     */
    public static final class Tuple1<T1> implements IValue1<T1> {
        private final T1 v0;

        /**
         * Create a new 1-tuple.
         */
        public Tuple1(T1 v0) {
            this.v0 = v0;
        }

        @Override
        public T1 _0() {
            return this.v0;
        }

    }


    /**
     * A type that has a 2nd element.
     */
    public static interface IValue2<T> {

        /**
         * Get the 2nd element.
         */
        T _1();
    }


    /**
     * A tuple of 2 elements.
     */
    public static final class Tuple2<T1, T2> implements IValue1<T1>, IValue2<T2> {
        private final T1 v0;
        private final T2 v1;

        /**
         * Create a new 2-tuple.
         */
        public Tuple2(T1 v0, T2 v1) {
            this.v0 = v0;
            this.v1 = v1;
        }

        @Override
        public T1 _0() {
            return this.v0;
        }

        @Override
        public T2 _1() {
            return this.v1;
        }

    }


    /**
     * A type that has a 3rd element.
     */
    public static interface IValue3<T> {

        /**
         * Get the 3rd element.
         */
        T _2();
    }


    /**
     * A tuple of 3 elements.
     */
    public static final class Tuple3<T1, T2, T3> implements IValue1<T1>, IValue2<T2>, IValue3<T3> {
        private final T1 v0;
        private final T2 v1;
        private final T3 v2;

        /**
         * Create a new 3-tuple.
         */
        public Tuple3(T1 v0, T2 v1, T3 v2) {
            this.v0 = v0;
            this.v1 = v1;
            this.v2 = v2;
        }

        @Override
        public T1 _0() {
            return this.v0;
        }

        @Override
        public T2 _1() {
            return this.v1;
        }

        @Override
        public T3 _2() {
            return this.v2;
        }

    }


    /**
     * A type that has a 4th element.
     */
    public static interface IValue4<T> {

        /**
         * Get the 4th element.
         */
        T _3();
    }


    /**
     * A tuple of 4 elements.
     */
    public static final class Tuple4<T1, T2, T3, T4>
        implements IValue1<T1>, IValue2<T2>, IValue3<T3>, IValue4<T4> {
        private final T1 v0;
        private final T2 v1;
        private final T3 v2;
        private final T4 v3;

        /**
         * Create a new 4-tuple.
         */
        public Tuple4(T1 v0, T2 v1, T3 v2, T4 v3) {
            this.v0 = v0;
            this.v1 = v1;
            this.v2 = v2;
            this.v3 = v3;
        }

        @Override
        public T1 _0() {
            return this.v0;
        }

        @Override
        public T2 _1() {
            return this.v1;
        }

        @Override
        public T3 _2() {
            return this.v2;
        }

        @Override
        public T4 _3() {
            return this.v3;
        }

    }


    /**
     * A type that has a 5th element.
     */
    public static interface IValue5<T> {

        /**
         * Get the 5th element.
         */
        T _4();
    }


    /**
     * A tuple of 5 elements.
     */
    public static final class Tuple5<T1, T2, T3, T4, T5>
        implements IValue1<T1>, IValue2<T2>, IValue3<T3>, IValue4<T4>, IValue5<T5> {
        private final T1 v0;
        private final T2 v1;
        private final T3 v2;
        private final T4 v3;
        private final T5 v4;

        /**
         * Create a new 5-tuple.
         */
        public Tuple5(T1 v0, T2 v1, T3 v2, T4 v3, T5 v4) {
            this.v0 = v0;
            this.v1 = v1;
            this.v2 = v2;
            this.v3 = v3;
            this.v4 = v4;
        }

        @Override
        public T1 _0() {
            return this.v0;
        }

        @Override
        public T2 _1() {
            return this.v1;
        }

        @Override
        public T3 _2() {
            return this.v2;
        }

        @Override
        public T4 _3() {
            return this.v3;
        }

        @Override
        public T5 _4() {
            return this.v4;
        }

    }


    /**
     * A type that has a 6th element.
     */
    public static interface IValue6<T> {

        /**
         * Get the 6th element.
         */
        T _5();
    }


    /**
     * A tuple of 6 elements.
     */
    public static final class Tuple6<T1, T2, T3, T4, T5, T6>
        implements IValue1<T1>, IValue2<T2>, IValue3<T3>, IValue4<T4>, IValue5<T5>, IValue6<T6> {
        private final T1 v0;
        private final T2 v1;
        private final T3 v2;
        private final T4 v3;
        private final T5 v4;
        private final T6 v5;

        /**
         * Create a new 6-tuple.
         */
        public Tuple6(T1 v0, T2 v1, T3 v2, T4 v3, T5 v4, T6 v5) {
            this.v0 = v0;
            this.v1 = v1;
            this.v2 = v2;
            this.v3 = v3;
            this.v4 = v4;
            this.v5 = v5;
        }

        @Override
        public T1 _0() {
            return this.v0;
        }

        @Override
        public T2 _1() {
            return this.v1;
        }

        @Override
        public T3 _2() {
            return this.v2;
        }

        @Override
        public T4 _3() {
            return this.v3;
        }

        @Override
        public T5 _4() {
            return this.v4;
        }

        @Override
        public T6 _5() {
            return this.v5;
        }

    }


    /**
     * A type that has a 7th element.
     */
    public static interface IValue7<T> {

        /**
         * Get the 7th element.
         */
        T _6();
    }


    /**
     * A tuple of 7 elements.
     */
    public static final class Tuple7<T1, T2, T3, T4, T5, T6, T7> implements IValue1<T1>,
        IValue2<T2>, IValue3<T3>, IValue4<T4>, IValue5<T5>, IValue6<T6>, IValue7<T7> {
        private final T1 v0;
        private final T2 v1;
        private final T3 v2;
        private final T4 v3;
        private final T5 v4;
        private final T6 v5;
        private final T7 v6;

        /**
         * Create a new 7-tuple.
         */
        public Tuple7(T1 v0, T2 v1, T3 v2, T4 v3, T5 v4, T6 v5, T7 v6) {
            this.v0 = v0;
            this.v1 = v1;
            this.v2 = v2;
            this.v3 = v3;
            this.v4 = v4;
            this.v5 = v5;
            this.v6 = v6;
        }

        @Override
        public T1 _0() {
            return this.v0;
        }

        @Override
        public T2 _1() {
            return this.v1;
        }

        @Override
        public T3 _2() {
            return this.v2;
        }

        @Override
        public T4 _3() {
            return this.v3;
        }

        @Override
        public T5 _4() {
            return this.v4;
        }

        @Override
        public T6 _5() {
            return this.v5;
        }

        @Override
        public T7 _6() {
            return this.v6;
        }

    }


    /**
     * A type that has a 8th element.
     */
    public static interface IValue8<T> {

        /**
         * Get the 8th element.
         */
        T _7();
    }


    /**
     * A tuple of 8 elements.
     */
    public static final class Tuple8<T1, T2, T3, T4, T5, T6, T7, T8> implements IValue1<T1>,
        IValue2<T2>, IValue3<T3>, IValue4<T4>, IValue5<T5>, IValue6<T6>, IValue7<T7>, IValue8<T8> {
        private final T1 v0;
        private final T2 v1;
        private final T3 v2;
        private final T4 v3;
        private final T5 v4;
        private final T6 v5;
        private final T7 v6;
        private final T8 v7;

        /**
         * Create a new 8-tuple.
         */
        public Tuple8(T1 v0, T2 v1, T3 v2, T4 v3, T5 v4, T6 v5, T7 v6, T8 v7) {
            this.v0 = v0;
            this.v1 = v1;
            this.v2 = v2;
            this.v3 = v3;
            this.v4 = v4;
            this.v5 = v5;
            this.v6 = v6;
            this.v7 = v7;
        }

        @Override
        public T1 _0() {
            return this.v0;
        }

        @Override
        public T2 _1() {
            return this.v1;
        }

        @Override
        public T3 _2() {
            return this.v2;
        }

        @Override
        public T4 _3() {
            return this.v3;
        }

        @Override
        public T5 _4() {
            return this.v4;
        }

        @Override
        public T6 _5() {
            return this.v5;
        }

        @Override
        public T7 _6() {
            return this.v6;
        }

        @Override
        public T8 _7() {
            return this.v7;
        }

    }


    /**
     * A type that has a 9th element.
     */
    public static interface IValue9<T> {

        /**
         * Get the 9th element.
         */
        T _8();
    }


    /**
     * A tuple of 9 elements.
     */
    public static final class Tuple9<T1, T2, T3, T4, T5, T6, T7, T8, T9>
        implements IValue1<T1>, IValue2<T2>, IValue3<T3>, IValue4<T4>, IValue5<T5>, IValue6<T6>,
        IValue7<T7>, IValue8<T8>, IValue9<T9> {
        private final T1 v0;
        private final T2 v1;
        private final T3 v2;
        private final T4 v3;
        private final T5 v4;
        private final T6 v5;
        private final T7 v6;
        private final T8 v7;
        private final T9 v8;

        /**
         * Create a new 9-tuple.
         */
        public Tuple9(T1 v0, T2 v1, T3 v2, T4 v3, T5 v4, T6 v5, T7 v6, T8 v7, T9 v8) {
            this.v0 = v0;
            this.v1 = v1;
            this.v2 = v2;
            this.v3 = v3;
            this.v4 = v4;
            this.v5 = v5;
            this.v6 = v6;
            this.v7 = v7;
            this.v8 = v8;
        }

        @Override
        public T1 _0() {
            return this.v0;
        }

        @Override
        public T2 _1() {
            return this.v1;
        }

        @Override
        public T3 _2() {
            return this.v2;
        }

        @Override
        public T4 _3() {
            return this.v3;
        }

        @Override
        public T5 _4() {
            return this.v4;
        }

        @Override
        public T6 _5() {
            return this.v5;
        }

        @Override
        public T7 _6() {
            return this.v6;
        }

        @Override
        public T8 _7() {
            return this.v7;
        }

        @Override
        public T9 _8() {
            return this.v8;
        }

    }

}
