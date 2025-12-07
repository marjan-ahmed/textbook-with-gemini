import React from 'react';
import Link from '@docusaurus/Link';
import styles from './Footer.module.css';

interface FooterLink {
  text: string;
  href: string;
}

interface FooterProps {
  copyrightText: string;
  links: FooterLink[];
}

const Footer: React.FC<FooterProps> = ({ copyrightText, links }) => {
  return (
    <footer className={styles.footer}>
      <div className="container">
        <div className={styles.footerContent}>
          <div className={styles.footerLinks}>
            {links.map((link, idx) => (
              <Link key={idx} to={link.href} className={styles.footerLink}>
                {link.text}
              </Link>
            ))}
          </div>
          <div className={styles.copyright}>{copyrightText}</div>
        </div>
      </div>
    </footer>
  );
};

export default Footer;
