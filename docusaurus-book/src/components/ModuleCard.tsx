import React from 'react';
import Link from '@docusaurus/Link';
import styles from './ModuleCard.module.css';

interface Module {
  title: string;
  focus: string;
  topics: string[];
  icon?: string;
  link: string;
}

interface ModuleCardProps {
  module: Module;
}

const ModuleCard: React.FC<ModuleCardProps> = ({ module }) => {
  return (
    <div className={styles.moduleCard}>
      <div className={styles.cardHeader}>
        {module.icon && (
          <div className={styles.moduleIcon}>
            <img src={module.icon} alt={module.title} />
          </div>
        )}
        <h3 className={styles.moduleTitle}>{module.title}</h3>
      </div>
      <p className={styles.moduleFocus}>{module.focus}</p>
      <ul className={styles.topicList}>
        {module.topics.map((topic, idx) => (
          <li key={idx}>{topic}</li>
        ))}
      </ul>
      <Link to={module.link} className={styles.moduleLink}>
        Explore Module →
      </Link>
    </div>
  );
};

export default ModuleCard;
