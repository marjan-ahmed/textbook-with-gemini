import React from 'react';
import styles from './WeeklyTimeline.module.css';

interface Week {
  range: string;
  topics: string[];
}

interface WeeklyTimelineProps {
  weeks: Week[];
}

const WeeklyTimeline: React.FC<WeeklyTimelineProps> = ({ weeks }) => {
  return (
    <section className={styles.timeline}>
      <div className="container">
        <h2 className="text--center margin-bottom--lg">Course Timeline</h2>
        <div className={styles.timelineContainer}>
          {weeks.map((week, idx) => (
            <div key={idx} className={styles.timelineItem}>
              <div className={styles.timelineMarker}>
                <div className={styles.markerDot}></div>
              </div>
              <div className={styles.timelineContent}>
                <h3 className={styles.weekRange}>{week.range}</h3>
                <ul className={styles.topicList}>
                  {week.topics.map((topic, topicIdx) => (
                    <li key={topicIdx}>{topic}</li>
                  ))}
                </ul>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
};

export default WeeklyTimeline;
