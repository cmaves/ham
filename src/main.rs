fn f() -> f64 {
    //0.0001220703125
    (-13.0_f64).exp2()
}
fn main() {
    let mut sum = 0.0;
    for _ in 0..500000 {
        let i = f();
        sum += i;
    }
    println!("{}", sum);
}
