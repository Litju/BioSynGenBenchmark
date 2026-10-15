"""RES-506 release manifest: frozen Git inventory plus explicit owned-scope policy.

Pattern screening is bounded assurance. This gate never grants source rights
by itself; the direct owner decision/NOTICE and inspected artifact provenance
are the authority for the documented personal-research distribution scope.
"""
import argparse
import hashlib
import json
from pathlib import Path
from scripts.audit_i3_provenance import audit, git

LICENSE_SHA256="cfc7749b96f63bd31c3c42b5c471bf756814053e847c10f3eb003417bc523d30"
PNG_PREFIXES=("runs/i3/res504/figure-previews/","runs/i3/res507/rendered/","runs/i3/res507/rendered-final/")


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ref",default="HEAD")
    parser.add_argument("--out",default="runs/i3/res506/release-inventory.json")
    args=parser.parse_args(argv)
    result=audit(args.ref)
    revision=result['revision']
    license_bytes=git('show',revision+':LICENSE')
    decision=json.loads(git('show',revision+':docs/i3/RES505_OWNER_DECISION.json'))
    problems=[]
    if hashlib.sha256(license_bytes).hexdigest()!=LICENSE_SHA256 or decision['license']!='Apache-2.0':
        problems.append("selected license/owner decision mismatch")
    for blob in result['history_blobs']:
        forbidden=set(blob['findings'])-{'machine_path','binary_manual_review'}
        if forbidden:
            problems.append({'paths':blob['paths'],'findings':sorted(forbidden)})
        if 'binary_manual_review' in blob['findings'] and not all(
                p.endswith('.png') and p.startswith(PNG_PREFIXES) for p in blob['paths']):
            problems.append({'paths':blob['paths'],'finding':'unknown binary requires manual rights/content review'})
    for item in result['files']:
        item['license']='Apache-2.0 (owned distribution scope; external source/data excluded)'
        item['disposition']='CLEARED_FOR_PUBLIC_REVIEW_UNDER_OWNER_DECLARED_SCOPE'
        if item['path'].startswith(('.claude/','.venv/','research/')):
            problems.append({'path':item['path'],'finding':'excluded local/private path present in tree'})
    result['issue']='RES-506'
    result['owner_decision']=decision
    result['publication_gate']={'passed':not problems,'problems':problems,
        'rights_basis':'Direct owner-selected Apache-2.0 and personal-research declaration; original source summaries and manufactured outputs inspected.',
        'metadata_disposition':'Canonical/generic operator paths retained after nonsecret review; original failures not rewritten.',
        'binary_disposition':'17 owned manufactured PNG previews/figures manually inspected; no participant photography or unlicensed media.',
        'limitations':'Pattern coverage and ownership declaration are bounded assurance, not a legal title certificate or universal secret detector.'}
    path=Path(args.out);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(result,indent=2,sort_keys=True)+'\n',encoding='utf-8',newline='\n')
    print(json.dumps({'revision':revision,'passed':not problems,'files':len(result['files']),
                      'commits':len(result['commit_ledger']),'history_blobs':len(result['history_blobs'])}))
    return 0 if not problems else 1


if __name__=='__main__':
    raise SystemExit(main())
