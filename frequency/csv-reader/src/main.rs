#![feature(iter_next_chunk)]

use std::{env, error::Error, fmt::Write, io::{BufRead, BufReader}, process::exit};
use indicatif::{ProgressBar, ProgressState, ProgressStyle};
use plotters::prelude::*;

use std::str;
const UPPER_LIMIT: f32 = 0.250;
const LOWER_LIMIT: f32 = 0.220;
const INTERVAL: f32 = 0.0000001;
const LINES: usize = 10_000;
const GRAPH_NAME: &str = "graph.png";

fn add_iterator_to_csv_result<T>(pb: &ProgressBar, result_vec: &mut Vec<f32>, iter: &[Result<String, T>]){
        for line in iter {
        pb.inc(1);
            if let Ok(string) = line {
                let split_line: Vec<&str> = string.split(',').collect();
                result_vec.push(split_line[1].parse().unwrap());
            }
        }
}

fn parse_csv(filename: &String) -> Result<Vec<f32>, Box<dyn Error>> {
    let mut file = std::fs::File::open(filename.clone()).unwrap();
    println!("Indexing file...");
    let mut buf = BufReader::new(&file);
    let count = buf.lines().count();
    
    file = std::fs::File::open(filename).unwrap();

    buf = BufReader::new(&file);

    let mut lines = buf.lines();
    
    let pb = ProgressBar::new(count as u64);
    pb.set_style(ProgressStyle::with_template("{spinner:.green} [{elapsed_precise}] [{wide_bar:.cyan/blue}] {pos}/{len} ({eta})")
        .unwrap()
        .with_key("eta", |state: &ProgressState, w: &mut dyn Write| write!(w, "{:.1}s", state.eta().as_secs_f64()).unwrap())
        .progress_chars("#>-"));
    lines.next();
    let mut result_vec: Vec<f32> = vec![];

    loop {
        let chunk = lines.next_chunk::<LINES>();
        if let Err(last) = chunk {
            add_iterator_to_csv_result(&pb, &mut result_vec, last.as_slice());
            break
        }

        let ok_chunk = chunk.unwrap();
        let slice_chunk = ok_chunk.as_slice();
        add_iterator_to_csv_result(&pb, &mut result_vec, slice_chunk);        

    }
         
    Ok(result_vec)
}

fn calculate_frequencies(values: Vec<f32>) -> (f32, Vec<(f32, u8, f32)>)  {
    let mut periods: Vec<(f32, u8, f32)> = vec![];
    let mut is_high = false;
    let mut current_stamps = 0;
    let mut max_stamps = 0;
    let mut begin_stamp = 0;

    for value in values {
        if is_high && value < LOWER_LIMIT {
            is_high = false;
            let period_value: (f32, u8, f32) = (begin_stamp as f32 * INTERVAL, 1, current_stamps as f32 * INTERVAL);
            periods.push(period_value);
            current_stamps = 0;
            begin_stamp = max_stamps;
        } else if !is_high && value > UPPER_LIMIT {
            is_high = true;
            let period_value: (f32, u8, f32) = (begin_stamp as f32 * INTERVAL, 0, current_stamps as f32 * INTERVAL);
            periods.push(period_value);
            current_stamps = 0;
            begin_stamp = max_stamps;
        }

        current_stamps += 1;
        max_stamps += 1;
    }

    let end_time = max_stamps as f32 * INTERVAL;

    (end_time, periods)
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() > 2 {
        println!("Invalid number of arguments.");
        exit(0);
    }
    let filename = &args[1];
    let parsed_csv = parse_csv(filename).unwrap();
    let (end_time, periods) = calculate_frequencies(parsed_csv);
    
    let mut values: Vec<(f32, f32)> = vec![];

    for (begin_time, _, period) in periods {
        values.push((begin_time, 1 as f32 / period));
    }

    let root = BitMapBackend::new(GRAPH_NAME, (1920, 1080)).into_drawing_area();
    root.fill(&WHITE).unwrap();
    let mut chart = ChartBuilder::on(&root)
        .caption("Frequency as a function of time", ("sans-serif", 30).into_font())
        .margin(5)
        .x_label_area_size(50)
        .y_label_area_size(50)
        .build_cartesian_2d(0f32..end_time+0.5, 0f32..1000f32).unwrap();

    chart.configure_mesh().draw().unwrap();

    chart
        .draw_series(LineSeries::new(
            (values).iter().map(|x| *x),
            &RED,
        )).unwrap();

    println!("Done! Saved as {}", GRAPH_NAME);
    root.present().unwrap();

}
