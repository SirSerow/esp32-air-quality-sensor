# Independent investigation of the spring TVOC increase

## Summary

The spring TVOC increase is present in the raw measurements, but it is not a smooth seasonal trend. It is mainly composed of repeated multi-hour and multi-day high-TVOC episodes, usually strongest overnight. The best-supported explanation is indoor VOC generation accumulating under limited ventilation. Missing ENS160 temperature/humidity compensation and a May/June device or operating change are important measurement confounders, but neither independently explains the full pattern.

The ENS160 is a broadband metal-oxide gas sensor. Its TVOC value is an algorithmic estimate for a mixture of gases, not a compound-specific laboratory measurement. AQI and eCO2 are derived from the same sensor signals and must not be treated as independent confirmation.

## Method

- Reprocessed [`sensor-readings-full.csv`](../data_analysis/sensor-readings-full.csv) directly.
- Used the notebooks only to locate the raw file; none of their cleaning results, aggregates, plots, or conclusions were reused.
- Converted timestamps to Japan time (`Asia/Tokyo`).
- Retained rows with a plausible epoch timestamp, `has_ens = 1`, `ens_validity = 0`, and a non-missing TVOC value.
- Excluded 270 startup records with non-epoch timestamps and all non-normal ENS160 validity periods.
- Analyzed 206,408 usable observations from February 16 through August 5, 2026. The normal sampling interval was approximately 61 seconds.
- Checked sample-weighted means, daily-balanced summaries, hourly profiles, high-TVOC episodes, missing periods, sensor warm-up events, firmware/storage behavior, and the bundled OpenAQ export.
- Independently verified the monthly means with both pandas and a standard-library streaming calculation.

## Monthly measurements

| Month | Observed days | Mean TVOC (ppb) | Median TVOC (ppb) | Samples at or above 1,000 ppb |
|---|---:|---:|---:|---:|
| February | 8, partial | 368 | 243 | 5.3% |
| March | 28 | 2,195 | 1,450 | 63.6% |
| April | 30 | 2,861 | 2,188 | 70.5% |
| May | 22, partial | 3,685 | 1,650 | 61.7% |
| June | 28, beginning June 3 | 528 | 374 | 11.8% |
| July | 31 | 269 | 182 | 2.0% |
| August | 5, partial | 584 | 356 | 13.0% |

February, May, June, and August do not represent complete calendar months. Monthly comparisons therefore describe the observed periods rather than an unbiased sample of every day in each month.

## Main observations

### The increase is episodic

March did not begin at a consistently high level. Daily medians were 334 ppb on March 4 and 443 ppb on March 6, followed by large episodes from March 8 onward.

Examples include:

- March 18-20: approximately 48 hours continuously above 1,000 ppb.
- May 17-20: approximately 72 hours above 660 ppb and 47 hours above 1,000 ppb.
- May 18: mean 23,839 ppb and median 19,752 ppb, with one reading at the ENS160 output ceiling of 65,000 ppb.

The May 17-20 episode strongly inflates the May average. Removing it lowers the May mean from 3,685 to 2,087 ppb. May remains elevated, but it is no longer an exceptional monthly maximum.

### Nighttime accumulation is prominent

For 68 of 79 spring days with comparable coverage, the median from 20:00-08:00 exceeded the median from 10:00-18:00. The median night/day ratio was 2.2.

The typical spring hourly TVOC profile fell sharply during the morning, reached its minimum around 11:00-13:00, and rose again during the evening and night. Temperature generally rose toward its afternoon maximum while TVOC fell, so simple warming cannot explain the daily pattern.

### Temperature and humidity are secondary factors

Mean measured temperature increased from 27.9 degrees C in February to 30.4 degrees C in May. Published chamber research shows that temperature, and sometimes humidity, can increase VOC emissions from building materials.

However:

- April and July had similar mean temperatures, approximately 29.2 and 29.1 degrees C, while July TVOC was approximately one-tenth of April TVOC.
- After removing month and hour-of-day patterns, rank correlations were only 0.104 with temperature and 0.054 with humidity.
- June and July humidity was higher than in spring while TVOC was much lower.

Temperature-dependent off-gassing may amplify some episodes, but the data do not support it as the sole cause.

## Ranked explanations

### 1. Indoor VOC accumulation under limited ventilation — strongest support

Repeated evening-to-morning increases, daytime declines, and events lasting one to three days are consistent with emissions accumulating in a closed room and being diluted when ventilation or occupancy changes.

Documented indoor VOC sources include paints, solvents, aerosols, cleaners, disinfectants, cosmetics, air fresheners, stored fuels, furnishings, adhesives, office supplies, and hobby materials. The US EPA notes that concentrations can remain elevated after an emitting activity ends and recommends increased ventilation.

### 2. Intermittent local VOC sources — strong but not identifiable

Cleaning, cooking, alcohol-containing products, sprays, fragrances, solvents, painting, adhesives, new furniture, or stored chemicals could cause the observed abrupt rises and slow declines. The May 17-20 event is particularly suggestive of an unusual local source or activity.

The ENS160 has broadband sensitivity to a mixture that can include ethanol, acetone, hydrogen, carbon monoxide, toluene, and oxidizing gases. It cannot determine which compound produced an episode.

### 3. Temperature-dependent material emissions — plausible contributor

Warmer conditions can increase emissions from some building materials and furnishings. This mechanism is documented experimentally, but similar April and July temperatures with very different TVOC levels show that it cannot explain the complete pattern.

### 4. Missing ENS160 temperature/humidity compensation — credible sensor bias

The firmware reads AHT temperature and humidity and then reads the ENS160 without writing the environmental values to the ENS160 compensation registers. See [`src/sensor_loop.c`](src/sensor_loop.c). The installed driver exposes compensation functions, but the application does not call them.

ScioSense documents that external temperature and humidity should be written to `TEMP_IN` and `RH_IN` for compensation. Its stated accuracy tests are primarily characterized around 25 degrees C and 50% RH. The omission weakens absolute and seasonal comparisons, although the observed weak correlations make it unlikely to explain the entire spring increase.

### 5. Power cycling, maintenance, relocation, or storage transition — important confound

The dataset contains 46 ENS160 power-on or warm-up sequences. For 34 events with adequate measurements before and after, the median two-hour post/pre TVOC ratio was 0.94. Power cycling therefore did not systematically raise TVOC.

Collection nevertheless changes materially:

- SD records end on May 22 at 13:24.
- A 12.4-day gap follows.
- NVS records begin on June 3.
- The firmware chooses SD or NVS storage according to SD availability at startup; see [`src/storage.c`](src/storage.c).

TVOC had already fallen to a daily median of 269 ppb on May 22, before the NVS period. Storage format itself is therefore unlikely to have caused the decrease. Associated actions such as SD removal, rebooting, cleaning, moving the sensor, changing its enclosure, or changing airflow remain plausible explanations for part of the spring-to-summer discontinuity.

### 6. Outdoor pollution or pollen — weak direct support

An independent comparison used the bundled [`openaq_measurements.csv`](../data_analysis/openaq_measurements.csv), containing one Yokohama monitoring location. Spring hourly rank correlations with indoor TVOC were:

| Outdoor measurement | Correlation with indoor TVOC |
|---|---:|
| NO | -0.126 |
| NO2 | 0.034 |
| NOx | 0.022 |
| PM2.5 | 0.146 |
| SO2 | -0.001 |

PM2.5 was higher in July than during spring while indoor TVOC was lowest. These pollutants do not reproduce the indoor seasonal pattern.

The station does not measure outdoor VOCs, ozone, PM10, pollen, or hyperlocal conditions, so those factors cannot be excluded. Direct pollen response is unlikely because the ENS160 senses gases rather than particles. Pollen could contribute indirectly through closed windows, altered ventilation, cleaning, or increased use of sprays and personal-care products.

## Measurement cautions

- ENS160 TVOC is an algorithmically processed broadband gas signal, not a chemical analysis of individual VOCs.
- The raw-data rank correlation between TVOC and ENS160 eCO2 is 1.000. eCO2 is therefore not independent supporting evidence.
- AQI divides the TVOC output into fixed bands and is also not independent evidence.
- Only one sensor was used; there is no colocated reference instrument.
- Activity, ventilation, window state, sensor position, maintenance, and firmware state were not logged.
- Large missing periods prevent a clean before/after seasonal comparison.
- The values should not be interpreted directly as compound-specific exposure or regulatory measurements.

## Recommended tests

1. Log window state, exhaust-fan use, cooking, cleaning, sprays, alcohol products, occupancy, and unusual activities.
2. Add a true NDIR CO2 sensor as an independent indicator of occupancy and ventilation.
3. Run two ENS160 sensors side by side: one updated with current AHT compensation every measurement cycle and one left at default compensation.
4. Log ENS160 `DATA_T`, `DATA_RH`, raw gas resistance, boot ID, reset reason, firmware hash, storage source, and sensor position.
5. During a high episode, open windows or run mechanical exhaust for 30 minutes. A rapid approximately exponential decline would support indoor accumulation.
6. Compare activated-carbon filtration with particle-only filtration. A response to carbon but not particle filtration would favor gases over pollen or particles.
7. Move a second sensor between rooms to determine whether the events are localized.
8. During an event, use a calibrated photoionization detector or sorbent sampling with GC/MS to identify and quantify actual compounds.

## Documentary sources

- ScioSense, [ENS160 Digital Metal-Oxide Multi-Gas Sensor Datasheet](https://www.sciosense.com/wp-content/uploads/2023/12/ENS160-Datasheet.pdf). Sensor principle, gas cross-sensitivity, baselining, environmental compensation, validity flags, warm-up behavior, and output range.
- US Environmental Protection Agency, [Volatile Organic Compounds' Impact on Indoor Air Quality](https://www.epa.gov/indoor-air-quality-iaq/volatile-organic-compounds-impact-indoor-air-quality). Common indoor sources, persistence after activities, and ventilation guidance.
- Zhou, S., Liu, H., Ding, Y., and Wu, Y. (2019), [The effects of temperature and humidity on the VOC emission rate from dry building materials](https://doi.org/10.1088/1757-899X/609/4/042001). Experimental evidence concerning environmental effects on emissions.
