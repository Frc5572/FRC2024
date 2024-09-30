use java_util::java_util;


java_util! {
    PowerDistributionVersion("edu/wpi/first/hal/PowerDistributionVersion") {
        // method example
        static fn (test,"run_test","(II)V"),
        // field example
        static (get_test, _, "test", "I")
    }
}

fn main() {
    test();
}